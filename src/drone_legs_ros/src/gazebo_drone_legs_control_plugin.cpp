#include <drone_legs_ros/gazebo_drone_legs_control_plugin.h>
#include <string>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include "drone_legs_ros/LegJointAngles.h"

namespace gazebo
{
    DroneLegsController::DroneLegsController()
    {
    }
    
    DroneLegsController::~DroneLegsController()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

        auto mgr = this->world_->GetPhysicsEngine()->GetContactManager();

        // Remove all contact sensors
        // This allows us to delete the model and spawn a new one within the same gazebo session
        // Failing to remove the sensors results in the following error:
        // "Filter with the same name already exists! Aborting"
        for (int s_i=0; s_i < this->contact_sensors_.size(); s_i++)
        {
            auto cs = this->contact_sensors_[s_i];
            
            // Reverse engineered from the definition in ContactSensor.cc
            // Unfortunately there's no better way. Ideally I'd get the filter name
            // from the Sensor class, but that is only introduced as a (private)
            // variable in versions after Gazebo 7.0
            std::string filterName = cs->ScopedName();
            
            // Remove the "default" from the name
            int pos = filterName.find("::", 0);
            filterName = filterName.substr(pos+2);
            boost::replace_all(filterName, "::", "/");
            
            // Finally remove the filter
            mgr->RemoveFilter(filterName);
        }
        
        this->node_handle_->shutdown();
        delete this->node_handle_;
        
        gzdbg << "Shut down DroneLegsController plugin" << std::endl;
    }


    /////////////////////////
    // load the controller
    void DroneLegsController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            gzerr << "Invalid joint count, DroneLegsController plugin not loaded\n";
            return;
        }
        
        
        // Store the model pointer for convenience.
        this->model_ = _model;
        this->world_ = this->model_->GetWorld();
        this->joints_ = this->model_->GetJoints();
        this->model_->GetJointController()->Reset();
        this->namespace_.clear();
        this->scope_prefix.clear();
        
        
        // Setup PID controllers
        bool success = true;
        success &= InitalizePID(_sdf, this->pid_position_base_hip, "pid_position_base_hip");
        success &= InitalizePID(_sdf, this->pid_position_thigh_lift, "pid_position_thigh_lift");
        success &= InitalizePID(_sdf, this->pid_position_knee_lift, "pid_position_knee_lift");
        if (!success)
            return;
            
        
        
        
        // Set state update rate
        if (!_sdf->HasElement("state_publish_rate"))
        {
            gzerr << "DroneLegsController plugin missing <state_publish_rate> element\n";
            return;
        }
        
        this->state_publish_interval = common::Time(0, 1000000000.0 / _sdf->GetElement("state_publish_rate")->Get<double>());
        this->last_state_publish = common::Time::GetWallTime();
        
        
        
        // Set sensor update rate
        if (!_sdf->HasElement("pressure_sensor_publish_rate"))
        {
            gzerr << "DroneLegsController plugin missing <pressure_sensor_publish_rate> element\n";
            return;
        }
        
        this->pressure_sensor_publish_interval = common::Time(0, 1000000000.0 / _sdf->GetElement("pressure_sensor_publish_rate")->Get<double>());
        this->last_pressure_sensor_publish = common::Time::GetWallTime();
        
        
        
        // Set up sensor filter information
        for (int i=0; i<4; i++)
        {
            this->last_contact_time.push_back(common::Time::GetWallTime());
            this->last_contact_force.push_back(0);
        }
        
        
        // Get contact sensors
        sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();
        sensors::Sensor_V sensors = mgr->GetSensors();
        for (int s_i=0; s_i<sensors.size(); s_i++)
        {
            auto sensor = sensors[s_i];
            
            // Make sure this is a contact sensor
            if( sensors::ContactSensorPtr cs = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor))
            {
                this->contact_sensors_.push_back(cs);
            }
        }
        
        // Activate all sensors
        for (int i=0; i<sensors.size(); i++)
        {
            auto sensor = sensors[i];
            sensor->SetActive(true);
        }
        
        
        
        
        // Get total weight of craft (to compute thrust later)
        this->mass_ = 0;
        this->links_ = this->model_->GetLinks();
        for (int i=0; i<this->links_.size(); i++)
        {
            try
            {
                this->mass_ += this->links_[i]->GetInertial()->GetMass();
            }
            catch(...){}
        }
            
        double gravity = -1 * this->model_->GetWorld()->Gravity()[2];
        this->weight_ = this->mass_ * gravity;
        
        
        // Set initial thrust
        if (_sdf->HasElement("initial_thrust"))
            this->thrust_ = this->weight_ * _sdf->GetElement("initial_thrust")->Get<double>();
        else
            this->thrust_ = 0;
        
        
        // Set up joint PIDs and initial positions as 0.0
        for (int i=0; i<this->joints_.size(); i++)
        {
            auto joint = this->joints_[i];
            
            //joint->SetVelocity(0, 0.0);
            //joint->SetForce(0, 0.0);
            //joint->SetParam("fmax", 0, 100.0);
            //joint->SetParam("vel", 0, 0.0);
            
            // Apply the P-controller to the joint based on the joint type (knee, thigh, base connection)
            std::string joint_name = joint->GetScopedName();
            std::vector<std::string> joint_types_vec = GetJointType(joint_name);
            if (joint_types_vec.empty())
            {
                gzerr << "Unknown joint type \"" << joint_name << "\". Failed to start plugin." << std::endl;
                return;
            }
            
            std::string joint_type = joint_types_vec[2];
            if (joint_type == "base_hip")
            {
                this->model_->GetJointController()->SetPositionPID(joint_name, this->pid_position_base_hip);
            }
            else if (joint_type == "thigh_lift")
            {
                this->model_->GetJointController()->SetPositionPID(joint_name, this->pid_position_thigh_lift);
            }
            else if (joint_type == "knee_lift")
            {
                this->model_->GetJointController()->SetPositionPID(joint_name, this->pid_position_knee_lift);
            }
            else
            {
                gzerr << "Unknown joint type \"" << joint_type << "\" in joint \"" << joint_name << "\". Failed to start plugin." << std::endl;
                return;
            }
            
            this->joint_types_.push_back(joint_types_vec);
            this->model_->GetJointController()->SetPositionTarget(joint_name, 0.0);
        }
        
        // Find the prefix to add to joint names for scoped names
        this->scope_prefix = this->joint_types_[0][0];
        
        
        
        
        // Initialize ros
        if (_sdf->HasElement("robotNamespace"))
            this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
            
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
        }
        
        this->node_handle_ = new ros::NodeHandle(this->namespace_);
        this->joint_command_service = this->node_handle_->advertiseService("leg_command", &DroneLegsController::JointCommandService, this);
        this->thrust_service = this->node_handle_->advertiseService("thrust_command", &DroneLegsController::VerticalThrustService, this);
        
        this->pub_pressure_sensor = this->node_handle_->advertise<drone_legs_ros::LegPressureSensors>("pressure_sensors", 50);
        this->pub_joint_angles = this->node_handle_->advertise<drone_legs_ros::LegJointAngles>("joint_angles", 50);
        
        // Start the update loop
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DroneLegsController::Update, this));
        
        gzmsg << "Loaded DroneLegsController plugin" << std::endl;
    }
    
    
    void DroneLegsController::Update()
    {
        //boost::mutex::scoped_lock scoped_lock(lock);
        common::Time curr_time = common::Time::GetWallTime();
    
    
    
        // Publish current state
        if (curr_time - this->last_state_publish >= this->state_publish_interval)
        {
            this->last_state_publish = curr_time;
            ros::Time ros_now = ros::Time::now();
            
            // Publish tf for base link
            physics::LinkPtr base_link =  this->model_->GetChildLink("base_link");
            auto pose_base = base_link->GetWorldPose();
                
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(pose_base.pos.x, pose_base.pos.y, pose_base.pos.z) );
            transform.setRotation( tf::Quaternion(pose_base.rot.x, pose_base.rot.y, pose_base.rot.z, pose_base.rot.w) );
            this->tf_br.sendTransform(tf::StampedTransform(transform, ros_now, "world", "drone_legs"));
            
            
            // Publish tf for links
            for (int i=0; i<this->links_.size(); i++)
            {
                auto link = this->links_[i];
                
                std::string parent_name = link->GetParent()->GetName();
                auto pose = link->GetRelativePose();
                
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) );
                transform.setRotation( tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w) );
                this->tf_br.sendTransform(tf::StampedTransform(transform, ros_now, parent_name, link->GetName()));
            }
            
            
            // Publish tf for sensors
            for (int s_i=0; s_i < this->contact_sensors_.size(); s_i++)
            {
                auto cs = this->contact_sensors_[s_i];
                auto pose = cs->Pose();
                
                // Scope name = World::Model::Link::Sensor
                std::string parent_name = cs->ParentName();
                std::vector<std::string> parent_vec = GetJointType(parent_name);
                parent_name = parent_vec[1]+ "_" + parent_vec[2];
                
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()) );
                transform.setRotation( tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()) );
                this->tf_br.sendTransform(tf::StampedTransform(transform, ros_now, parent_name, parent_vec[1]+ "_contact_sensor"));
                this->tf_br.sendTransform(tf::StampedTransform(transform, ros_now, parent_name, parent_vec[1]+ "_toe"));
            }
            
            
            // Publish joint angles
            if (this->pub_joint_angles.getNumSubscribers() > 0)
            {
                drone_legs_ros::LegJointAngles msg_angles;
                for (int i=0; i<this->joints_.size(); i++)
                {
                    auto joint = this->joints_[i];
                    auto name_vec = this->joint_types_[i];
                    
                    drone_legs_ros::JointAngle datum;
                    datum.leg = name_vec[1];
                    datum.type = name_vec[2];
                    datum.angle = joint->GetAngle(0).Radian();
                    
                    msg_angles.data.push_back(datum);
                }
                this->pub_joint_angles.publish(msg_angles);
            }
        }
        
    
    
    
    
    
    
    
    
    
    
    
        // Publish pressure sensor readings
        if (curr_time - this->last_pressure_sensor_publish >= this->pressure_sensor_publish_interval)
        {
            this->last_pressure_sensor_publish = curr_time;
            
            if (this->pub_pressure_sensor.getNumSubscribers() > 0)
            {
                drone_legs_ros::LegPressureSensors msg_pressure;
                
                for (int s_i=0; s_i < this->contact_sensors_.size(); s_i++)
                {
                    auto cs = this->contact_sensors_[s_i];
                    
                    msgs::Contacts contacts = cs->Contacts();
                    
                    ignition::math::Vector3d total_force(0,0,0);
                    
                    
                    // GetContacts returns all contacts on the collision body
                    unsigned int contactsPacketSize = contacts.contact_size();
                    for (unsigned int i = 0; i < contactsPacketSize; ++i)
                    {
                        unsigned int contactGroupSize = contacts.contact(i).position_size();
                        for (unsigned int j = 0; j < contactGroupSize; ++j)
                        {
                            auto force_vec = contacts.contact(i).wrench(j).body_1_wrench().force();
                            total_force += ignition::math::Vector3d(force_vec.x(), force_vec.y(), force_vec.z());
                        }
                    }
                    
                    double force = total_force.Length();
                    std::string sensor_name = cs->Name();
                    
                    // If the sensor momentarily lost contact with the ground, ignore it
                    if (force == 0)
                    {
                        if (curr_time - this->last_contact_time[s_i] >= this->lost_contact_threshold)
                        {
                            this->last_contact_force[s_i] = 0;
                        }
                        else
                        {
                            force = this->last_contact_force[s_i];
                        }
                    }
                    else
                    {
                        this->last_contact_time[s_i] = curr_time;
                        this->last_contact_force[s_i] = force;
                    }
                    
                    
                    if (sensor_name == "front_right_contact")
                        msg_pressure.front_right = force;
                    else if (sensor_name == "front_left_contact")
                        msg_pressure.front_left = force;
                    else if (sensor_name == "back_right_contact")
                        msg_pressure.back_right = force;
                    else if (sensor_name == "back_left_contact")
                        msg_pressure.back_left = force;
                }
                
                this->pub_pressure_sensor.publish(msg_pressure);
            }
        }
        
        
        
        // Give the model some upwards thrust
        physics::LinkPtr link=  this->model_->GetChildLink("base_link");
        
        // Add a force along the global z-axis
        link->AddForce(math::Vector3(0, 0, this->thrust_));
        
        // Add a force along the z-axis of base_link (ie. accounting for orientation)
        //link->AddRelativeForce(math::Vector3(0, 0, this->thrust_));
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    // Reset the controller
    void DroneLegsController::Reset()
    {
    }
    
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DroneLegsController)
    
    
    
    
    
    
    
    bool DroneLegsController::JointCommandService(
        drone_legs_ros::LegCommand::Request  &request,
        drone_legs_ros::LegCommand::Response &response)
    {
        std::vector<std::string> names = request.joint_names;
        std::vector<double> angles = request.joint_positions;
        
        if (names.size() != angles.size())
        {
            response.success = false;
            response.status_message = "[joint_names] and [joint_positions] do not have the same number of elements.";
            return false;
        }
        
        
        
        for (int i=0; i<names.size(); i++)
        {
            std::string name = names[i];
            double angle = angles[i];
            
            std::string scoped_name = this->scope_prefix + "::" + name;
            this->model_->GetJointController()->SetPositionTarget(scoped_name, angle);  
        }
        
        response.success = true;
        response.status_message = std::to_string(names.size()) + " joints set successfully.";
        
        //gzmsg << "Got a request" << std::endl;
        return true;
    }
    
    
    
    
    
    bool DroneLegsController::VerticalThrustService(
        drone_legs_ros::LegVerticalThrust::Request  &request,
        drone_legs_ros::LegVerticalThrust::Response &response)
    {
        this->thrust_ = request.thrust * this->weight_;
        
        //gzdbg << "Thrust = " << this->thrust_ << std::endl;
        
        response.success = true;
        response.status_message = "Thrust successfully set.";
        
        return true;
    }
    
    
    
    
    
    
    std::vector<std::string> DroneLegsController::GetJointType(std::string name)
    {
        // Explode the name into 3 components:
        // eg. drone_legs::front_left_knee_lift -> [drone_legs, front_left, knee_lift]
        
        std::vector<int> positions; // holds all the positions that sub occurs within str
        std::vector<std::string> final_vec;

        int pos = name.find("::", 0);
        positions.push_back(pos);

        pos = name.find("_", pos);
        bool valid = false;
        while(pos != std::string::npos)
        {
            if (valid)
            {
                positions.push_back(pos);
                break;
            }
            
            pos = name.find("_",pos+1);
            valid = ~valid;
        }
        
        // Are there enough elements?
        if (positions.size() != 2)
        {
            return final_vec;
        }
        
        
        // Explode string
        int count;
        
        final_vec.push_back( name.substr(0, positions[0]) );
        final_vec.push_back( name.substr(positions[0]+2, positions[1]-positions[0]-2) );
        final_vec.push_back( name.substr(positions[1]+1) );
        
        // Convert to lower case
        //std::transform(joint_type.begin(), joint_type.end(), joint_type.begin(), ::tolower);
        
        return final_vec;
    }
    
    
    bool DroneLegsController::InitalizePID(sdf::ElementPtr _sdf, common::PID& pid, std::string pid_name)
    {
        if (!_sdf->HasElement(pid_name))
        {
            gzerr << "DroneLegsController plugin missing <" << pid_name << "> element\n";
            return false;
        }
                
        std::string params = _sdf->GetElement(pid_name)->Get<std::string>();



        // Get the values
        double P, I, D;
        try {
            int offset = 0;
            std::string::size_type sz;
            
            P = std::stod (params,&sz); offset += sz;
            I = std::stod (params.substr(offset), &sz); offset += sz;
            D = std::stod (params.substr(offset), &sz); offset += sz;
        }
        catch (const std::invalid_argument& ia) {
            gzerr << "<" << pid_name << "> does not contain 3 valid doubles (value: " << params << ")" << std::endl;
            return false;
        }
        
        pid = common::PID(P, I, D);
        return true;
    }
}
#ifndef DRONE_LEGS_CONTROLLER_HH
#define DRONE_LEGS_CONTROLLER_HH

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

//#include <gazebo/common/Plugin.hh>
//#include <gazebo/physics/Collision.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <std_msgs/String.h>

// Services
#include "drone_legs_ros/LegCommand.h"
#include "drone_legs_ros/LegVerticalThrust.h"

// Messages
#include "drone_legs_ros/LegPressureSensors.h"
#include "drone_legs_ros/LegJointAngles.h"

namespace gazebo
{
  class DroneLegsController : public ModelPlugin
  {
  
    public:
        DroneLegsController();
        virtual ~DroneLegsController();
    
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Update();
        virtual void Reset();
        
    private:
        virtual std::vector<std::string> GetJointType(std::string);
        virtual bool InitalizePID(sdf::ElementPtr _sdf, common::PID& pid, std::string pid_name);
        virtual bool JointCommandService(drone_legs_ros::LegCommand::Request  &req, drone_legs_ros::LegCommand::Response &res);
        virtual bool VerticalThrustService(drone_legs_ros::LegVerticalThrust::Request  &request, drone_legs_ros::LegVerticalThrust::Response &response);
    
    private:
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::Link_V  links_;
        physics::Joint_V joints_;
        std::vector<std::vector<std::string> > joint_types_;
        std::vector<sensors::ContactSensorPtr> contact_sensors_;
        std::string scope_prefix;
        
        common::Time state_publish_interval;
        common::Time last_state_publish;
        common::Time pressure_sensor_publish_interval;
        common::Time last_pressure_sensor_publish;
        
        ros::NodeHandle* node_handle_;
        std::string namespace_;
        ros::ServiceServer joint_command_service;
        ros::ServiceServer thrust_service;
        ros::Publisher pub_pressure_sensor;
        ros::Publisher pub_joint_angles;
        tf::TransformBroadcaster tf_br;
        
        sensors::ContactSensor* cs_fr;
        
        double mass_;
        double weight_;
        double thrust_;
        
        std::vector<common::Time> last_contact_time;
        common::Time lost_contact_threshold = common::Time(0, 0.5 * 1000000000);
        std::vector<double> last_contact_force;
        
        
        boost::mutex lock;
        
        event::ConnectionPtr update_connection_;
        
        common::PID pid_position_base_hip;
        common::PID pid_position_thigh_lift;
        common::PID pid_position_knee_lift;
  };
}
#endif
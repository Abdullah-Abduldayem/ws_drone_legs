## Table of Contents
- [Setup](#setup)


## Setup

### Operating System
This runs on a debian operating system for the RaspberryPi. [DietPi](https://dietpi.com/) was installed. I had a few issues with it out of the box, so this is how I fixed it:

```
echo "deb https://archive.raspbian.org/raspbian stretch main contrib non-free" > /etc/apt/sources.list.d/raspi.list
wget https://archive.raspbian.org/raspbian.public.key -O - | apt-get add -
apt-get update
apt clean
G_AGUP
G_AGUG
apt-get install bash-doc
reboot
```

### Install dependencies
```
apt-get install build-essential git
apt-get install python3 python3-pip
pip3 install pyserial pyyaml
##pip3 install pyssc32
git clone https://github.com/Abdullah-Abduldayem/pyssc32.git
cd pyssc32
sudo python3 setup.py install
```

### SSH Key Setup for Development (Optional)

To allow for easy assess, we'll set up SSH access on both the RPi and the host machine.

On the development machine (eg. laptop), enter the following:
```
ssh-keygen -t rsa
ssh-add
```
If the default locations are used, the public key will be generated in `~/.ssh/id_rsa.pub` while the private key is in `~/.ssh/id_rsa`

The next step is to copy the key to the RPi. Replace the username and IP with those for the RPi.

```
ssh-copy-id root@192.168.1.22
```

### Installing on RPi


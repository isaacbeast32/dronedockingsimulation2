#https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md
#open ubuntu
#run following commands

___________________________________________________________________________

#install basic dependencies
sudo apt update
sudo apt install git wget curl build-essential cmake python3-pip xterm

___________________________________________________________________________

#install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full #unable to install

#add ROS to shell
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

___________________________________________________________________________

#set up catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

#add workspace to shell
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

___________________________________________________________________________

#clone iq_sim folder
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
git clone https://github.com/Intelligent-Quads/iq_tutorials.git

___________________________________________________________________________
#install ardupilot + gazebo plugin (should already have this)
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install

___________________________________________________________________________

#build iq_sim
cd ~/catkin_ws
catkin_make
source devel/setup.bash





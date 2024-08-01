#!/bin/bash

# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|


# Welcome! this is the BPP ROS2 kickstarter script that will install all the necessary pacakges, libraries, software, interfaces, and assorted
# utilits that are required to run our software! Happy coding :D


# authors: Rahul Vishnoi, Romeo Perlstein, <your name here>

# for setting certain colors, for eventually adding SSL banner

trap code_killer SIGINT

code_killer(){
    echo "\nctrl-c input received, stopping kickstarter..."
    kill 0
}


Red='\033[0;31m'
Blue='\033[0;34m'
Green='\033[0;32m'
White='\033[0;37m'

UBlue='\033[4;34m'

color_off='\033[0m'

# ascii art from https://ascii-generator.site/
# good color and bold referce from https://stackoverflow.com/questions/4414297/unix-bash-script-to-embolden-underline-italicize-specific-text

echo -e "---------- ${White}Welcome to the BOBOT kickstarter!${color_off} -----------"
echo -e "------- ${Red}(WARNING: its gonna take a few minutes)${color_off} --------"

echo -e "${Green}prompting for password, this is necessary for updating, upgrading, and installing necessary components!${color_off}"
echo

sudo apt update
sleep 1
sudo apt upgrade -y

##### ----- #####
#install ros2 (this way for now, may change)
echo -e "${Green}\nInstalling ROS2 Jazzy Jalisco! This will take several minutes... \n"

#install locales, if not present (TODO: make this optional by havig the bashscript automatically check your locales)
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US.UTF-8 

#set up your source repos
sudo apt install software-properties-common -y
#set your system to use Ubuntu Universe repo
sudo add-apt-repository universe
#add the ROS2 repository to your system, including the key-ing setup that it uses
sudo apt update && sudo apt install curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
#update once more and install!
sudo apt update
sudo apt install -y \
    ros-jazzy-rmw-fastrtps-cpp
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
##### ----- #####

# install colcon build tools and other thingies
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-pip -y
pip3 install setuptools
pip3 install xacro
sudo apt install python3-rosdep -y

echo -e "\n${Red}NOTICE:${color_off} Do you want to update rosdep? ${Red}updating rosdep can take a while, and it's not recommeneded to do it frequently${color_off}"
echo -e "(MANDATORY if this is your first time running the kickstarter, recommended if its been a while since you've updated it)\n"
echo -e "Update rosdep? (yes/no)"
read rosdep_request
if [ ${rosdep_request} == "Yes" ] || [ ${rosdep_request} == "yes" ] || [ ${rosdep_request} == "YES" ] || [ ${rosdep_request} == "y" ]
    then
    echo -e ${Green}Updating rosdep, this is gonna take a while, please be patient!${color_off}
    sudo rosdep init
    rosdep update
    else
    echo -e "\nskipping rosdep update\n"
fi
rosdep install --from-paths src -y --ignore-src --rosdistro jazzy

# TODO - POssible move the ros dependencies to another tools file? not critical but could be better than directly modifiying the kickstarter for new people

# its mission critical trust fr fr on god no cap
sudo apt install sl -y

sudo apt update -y
sudo apt upgrade -y
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
colcon build
source install/setup.bash
echo -e "\n${Green}Finished installing! Be sure to check for errors and out-of-date utilities\nFor any issues, please file a Git-issue or contact Rahul Vishnoi${color_off}\n"

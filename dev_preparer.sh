#!/bin/bash

# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|

# This is a dev preparer tool. All it does is turn off the bobot_bootup sequence so you can just develope schtuff

# Pretty color variables
Red='\033[0;31m'
Blue='\033[0;34m'
Green='\033[0;32m'
White='\033[0;37m'
color_off='\033[0m'


echo -e "\n\tPreparing workspace for development! Make sure this\n\ttool is run from the bobot workspace"
echo -e "\tPeruvian food is still better cope cope cope\n"

echo -e "${Red}\tDisabling bobot_bootup sequence - software will no longer run on bootup!${color_off}"
echo -e "\tMake sure to run 'flight_preparer.sh' before launch\n"

# create an environment variable
echo -e "/**:\n\tros__parameters:\n\t\tIS_FLIGHT: false" > src/bobot_launcher/bobot_launch_config.yaml

echo -e "[Desktop Entry]\nType=Application\nExec=""\nHidden=false\nNoDisplay=true\nX-GNOME-Autostart-enabled=true\nName[en_US]=bobot_bootup\nName=bobot_bootup\nComment[en_US]=kys\nComment=SSL ROCKS" > ~/.config/autostart/bobot_bootup.desktop



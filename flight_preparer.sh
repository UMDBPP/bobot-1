#!/bin/bash

# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|


# This is the flight preparer tool - all it does is erase whatever was in timer.txt and abs_position.txt
# Thats literally all it does

# Pretty color variables
Red='\033[0;31m'
Blue='\033[0;34m'
Green='\033[0;32m'
White='\033[0;37m'
color_off='\033[0m'


echo -e "\n\tPreparing workspace for flight! Make sure this tool is run from the bobot workspace"
echo -e "\tJJ if you are reading this Peruvian food is better than Pho stay MAD\n"

# Empty the timer data file
echo -e "${Red}\tErasing timer data from:${color_off} src/bobot_bin/timer.txt"
> src/bobot_bin/timer.txt
echo -e "${Green}\tFile contents erased from:${color_off} src/bobot_bin/timer.txt\n"

# Empty the absolute position data for the extension motor
echo -e "${Red}\tErasing absolute position data from:${color_off} src/bobot_bin/abs_position.txt"
> src/bobot_bin/abs_position.txt
echo -e "${Green}\tFile contents erased from:${color_off} src/bobot_bin/abs_position.txt\n"

# Empty the reset counter before a flight
echo -e "\t${Red}clearing reset counter:${color_off} src/bobot_bin/reset_counter.txt"
> reset_counter.txt
echo -e "\t${Green}reset counter cleared!${color_off}\n"

# Empty the current flight data before a flight
echo -e "\t${Red}clearing previous flight data buffer:${color_off} src/bobot_bin/reset_counter.txt"
> bobot_recovery/current_flight.txt
echo -e "\t${Green}flight data buffers clear and ready!!${color_off}\n"

# Clear the log files before we start the flight
echo -e "\tClear Flight Log and Error Log data?(Yes/no)"
read clear_log_request
if [ ${clear_log_request} == "Yes" ] || [ ${clear_log_request} == "yes" ] || [ ${clear_log_request} == "YES" ] || [ ${clear_log_request} == "y" ]
    then
    rm src/bobot_bin/log_files/error_logs/*
    rm src/bobot_bin/log_files/flight_logs/*
    echo -e "${Green}\tLog data cleared!${color_off}\n"
fi

# Set up the system to boot on startup
echo -e "\tSetting up program to run on boot-up"
startup_script="/tools/bobot_bootup.sh"
path=$(realpath "$0")
DIR=$(dirname "$path")
FULLPATH=${DIR}${startup_script}

# create an environment variable
export IS_FLIGHT=true

echo -e "[Desktop Entry]\nType=Application\nExec=${FULLPATH}\nHidden=false\nNoDisplay=true\nX-GNOME-Autostart-enabled=true\nName[en_US]=bobot_bootup\nName=bobot_bootup\nComment[en_US]=kys\nComment=SSL ROCKS" > ~/.config/autostart/bobot_bootup.desktop

echo -e "${Green}\n\tDone! Bobot software is ready for flight, happy travels :D\n${color_off}"
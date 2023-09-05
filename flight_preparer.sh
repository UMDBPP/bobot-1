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


echo -e "\n\tPreparing workspace for flight!"
echo -e "\tJJ if you are reading this Peruvian food is better than Pho stay MAD\n"
echo -e "${Red}\tErasing timer data from:${color_off} src/bobot_bin/timer.txt"

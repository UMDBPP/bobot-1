// |***************************************************************|
// |* (c) Copyright 2023                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#ifndef BOBOT_UTILS__BOBOT_COMMON_TYPES_HPP
#define BOBOT_UTILS__BOBOT_COMMON_TYPES_HPP

#include <vector>
#include <string>

#pragma once // This allows me to be a bad programmer and define libraries in any files that ALSO use this file, without causing compiler issues

namespace bobot
{



    // Make a structure to hold all of our pin information
    struct pin_info{
        std::vector<std::string> pins;
    } typedef PinInfo; // Even though this is C syntax, better to be safe than sorry, no?



}




#endif //BOBOT_UTILS__BOBOT_COMMON_TYPES_HPP
//
// Created by francesco on 17.05.21.
//

#ifndef BT_ROBOTICS_AUXILLARY_H
#define BT_ROBOTICS_AUXILLARY_H

#include <iostream>
#include <string>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "json.h"

namespace auxillary {

// Get current date/time, format is MM-DD.HH:mm:ss
std::string currentDateTime();

}

#endif //BT_ROBOTICS_AUXILLARY_H

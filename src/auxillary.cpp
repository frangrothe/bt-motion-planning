//
// Created by francesco on 17.05.21.
//

#include "auxillary.h"

namespace auxillary {

std::string currentDateTime() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m__%H-%M-%S");
    return oss.str();

}
}



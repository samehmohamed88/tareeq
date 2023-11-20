#pragma once

#include "commaai/can_client.h"

#include <string>
#include <vector>

bool safety_setter_thread(std::vector<Panda *> pandas);
void boardd_main_thread(std::vector<std::string> serials);

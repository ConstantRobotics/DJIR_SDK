#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "DJIR_SDK.h"
using namespace DJIR_SDK;

bool print_until_arrives(DJIRonin* gimbal, int16_t yaw, int16_t roll, int16_t pitch);

int main(void)
{
    std::cout << "###########################################" << std::endl;
    std::cout << "#                                         #" << std::endl;
    std::cout << "#          DJIR-SDK Test v1.0.0           #" << std::endl;
    std::cout << "#                                         #" << std::endl;
    std::cout << "###########################################" << std::endl;

    DJIRonin gimbal = DJIRonin();

    // Connect to DJI Ronin Gimbal
    gimbal.connect();

    // Select ABSOLUTE_CONTROL mode
    gimbal.set_move_mode(MoveMode::ABSOLUTE_CONTROL);

    // Move to center position (yaw = 0, roll = 0, pitch = 0) for 2000ms
    gimbal.move_to(0, 0, 0, 2000);
    while (print_until_arrives(&gimbal, 0, 0, 0));

    // Move to first test point (yaw = -1200, roll = 0, pitch = 300) for 2000ms
    gimbal.move_to(-1200, 0, 300, 2000);
    while (print_until_arrives(&gimbal, -1200, 0, 300));

    // Move to second test point (yaw = 1200, roll = 0, pitch = -300) for 2000ms
    gimbal.move_to(1200, 0, -300, 2000);
    while (print_until_arrives(&gimbal, 1200, 0, -300));


    // Move to center position (yaw = 0, roll = 0, pitch = 0) for 2000ms
    gimbal.move_to(0, 0, 0, 2000);
    while (print_until_arrives(&gimbal, 0, 0, 0));

    std::cout << "Press any key to continie...";
    getchar();
    return 1;
}

bool print_until_arrives(DJIRonin* gimbal, int16_t yaw, int16_t roll, int16_t pitch)
{
    int16_t _yaw = 0; int16_t _roll = 0; int16_t _pitch = 0;
    gimbal->get_current_position(_yaw, _roll, _pitch);
    std::cout << "yaw = " << _yaw
              << " roll = " << _roll
              << " pitch = " << _pitch <<std::endl;

    uint16_t delta = 20;
    return !((_yaw >= yaw - delta &&  _yaw <= yaw + delta) &&
             (_roll >= roll - delta &&  _roll <= roll + delta) &&
             (_pitch >= pitch - delta &&  _pitch <= pitch + delta));
}

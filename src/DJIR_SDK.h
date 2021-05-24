#ifndef DJI_R_SDK_H
#define DJI_R_SDK_H

#include <stdlib.h>
#include <stdint.h>
#include <vector>

#if (defined _WIN32 && defined RF62X_LIBRARY)
#define API_EXPORT __declspec(dllexport)
#else
#define API_EXPORT
#endif

namespace DJIR_SDK {

enum class ReturnCode {
    EXECUTION_SUCCESSFUL = 0,
    PARSE_ERROR = 1,
    EXECUTION_FAILS = 2
};

enum class AxisType {
    YAW = 0,
    ROLL = 1,
    PITCH = 2
};

enum class MoveMode {
    INCREMENTAL_CONTROL = 0,
    ABSOLUTE_CONTROL = 1
};

enum class SpeedControl {
    DISABLED = 0,
    ENABLED = 1
};

enum class FocalControl {
    ENABLED = 0,
    DISABLED = 1
};



API_EXPORT class DJIRonin
{
public:
    DJIRonin();
    ~DJIRonin();

    /**
     * @brief connect - Connect to DJI Ronin device
     * @return True if success
     */
    bool connect();
    /**
     * @brief disconnect - Disconnect from DJI Ronin device
     * @return True if success
     */
    bool disconnect();

    /**
     * @brief move_to - Handheld Gimbal Position Control (p.5, 2.3.4.1)
     * @param yaw Yaw angle, unit: 0.1° (range: -1800 to +1800)
     * @param roll Roll angle, unit: 0.1° (range: -1800 to +1800)
     * @param pitch Pitch angle, unit: 0.1° (range: -1800 to +1800)
     * @param time_ms Command execution speed, unit: ms. Min value is 100ms.
     * Time is used to set motion speed when gimbal is executing this command.
     * @return True if success
     */
    bool move_to(int16_t yaw, int16_t roll, int16_t pitch, uint16_t time_ms);

    /**
     * @brief set_inverted_axis - Handheld Gimbal Position Control (p.5, 2.3.4.1)
     * @param axis Type of axis (YAW, ROLL, PITCH)
     * @param invert True if invert
     * @return True if success
     */
    bool set_inverted_axis(AxisType axis, bool invert);

    /**
     * @brief set_move_mode Set move mode
     * @param type INCREMENTAL or ABSOLUTE
     * @return True if success
     */
    bool set_move_mode(MoveMode type);

    /**
     * @brief set_speed - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
     * @details This command can only control for 0.5s each time it is issued
     * due to safety reasons. If users require continuous speed, they can send
     * this command periodically. If users want to stop the rotation of three
     * axes immediately, they can set the fields of yaw, pitch, and roll in
     * set_speed_mode method as 0.
     * @param yaw Unit: 0.1°/s (range: 0°/s to 360°/s)
     * @param roll Unit: 0.1°/s (range: 0°/s to 360°/s)
     * @param pitch Unit: 0.1°/s (range: 0°/s to 360°/s)
     * @return True if success
     */
    bool set_speed(uint16_t yaw, uint16_t roll, uint16_t pitch);

    /**
     * @brief set_speed_mode - Handheld Gimbal Speed Control (p.6, 2.3.4.2)
     * @param speed_type Enable or Disable speed control
     * @param focal_type Enable or Disable the impact of camera focal length
     * into consideration
     * @return True if success
     */
    bool set_speed_mode(SpeedControl speed_type, FocalControl focal_type);


    /**
     * @brief get_current_position - Get Gimbal Information (p.6, 2.3.4.3)
     * @param yaw Yaw axis angle (unit: 0.1°)
     * @param roll Roll axis angle (unit: 0.1°)
     * @param pitch Pitch axis angle (unit: 0.1°)
     * @return True if success
     */
    bool get_current_position(int16_t& yaw, int16_t& roll, int16_t& pitch);

private:
    void* _can_conn;
    void* _pack_thread;
    uint8_t _position_ctrl_byte;
    uint8_t _speed_ctrl_byte;
    void* _cmd_cmb;
};


}


#endif //DJI_R_SDK_H

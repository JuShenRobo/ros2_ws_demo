#pragma once

#include <rpos/core/rpos_core_config.h>

#include <cstdint>

namespace rpos { namespace core {

    struct RPOS_CORE_API SlamcoreShutdownParam
    {
        std::uint32_t restartTimeIntervalMinute;
        std::uint32_t shutdownTimeIntervalMinute;
        bool resetShutdownRestartTimeInterval;

        SlamcoreShutdownParam();
    };

    struct RPOS_CORE_API SetChargeControl
    {
        std::uint32_t chargeControl;    //=1: disable charging; =0: enable charging

        SetChargeControl();
    };

    struct RPOS_CORE_API GetChargeControlStatus
    {
        std::uint32_t chargeControlStatus;  //=1: disabled charging; =0: enabled charging

        GetChargeControlStatus();
    };

    struct RPOS_CORE_API ImuTreshTestData
    {
        float check_cov_roll;
        float check_cov_pitch;
        float check_cov_yaw;
        float gyro_cov_roll;
        float gyro_cov_pitch;
        float gyro_cov_yaw;
        std::uint32_t thresh;
        float sum_yaw;
        std::uint32_t time_stamp;

        ImuTreshTestData();
    };

    struct RPOS_CORE_API IMURawADCData
    {
        int acc_x;
        int acc_y;
        int acc_z;
        int gyro_x;
        int gyro_y;
        int gyro_z;
        int comp_x;
        int comp_y;
        int comp_z;

        uint32_t timestamp;
        IMURawADCData();
    };

    struct RPOS_CORE_API IMURawStandardUnitData
    {
        float acc_x;
        float acc_y;
        float acc_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float comp_x;
        float comp_y;
        float comp_z;

        uint32_t timestamp;
        IMURawStandardUnitData();
    };

    enum LightChannel : std::uint8_t
    {
        LightChannelOne = 0,
        LightChannelTwo = 1
    };

    enum LightControlPart : std::uint8_t
    {
        LightControlPartLeft = 0,
        LightControlPartRight = 1
    };

    enum LightControlMode : std::uint8_t
    {
        LightControlModeAlwaysBright = 0,
        LightControlModeBreathe = 1,
        LightControlModeBlink = 2,
        LightControlModeHorseLamp = 3
    };

    struct RPOS_CORE_API LightColor
    {
        std::uint8_t red;
        std::uint8_t green;
        std::uint8_t blue;

        LightColor();
        LightColor(std::uint8_t red, std::uint8_t green, std::uint8_t blue);
    };

    struct RPOS_CORE_API LightControlData
    {
        LightChannel channel;
        LightControlPart controlPart;
        LightControlMode mode;
        LightColor color;
        LightColor brightnessEndColor;
        std::uint16_t brightMs;
        std::uint16_t offMs;

        LightControlData();
        LightControlData(LightChannel channel, LightControlPart controlPart, LightControlMode mode,
                         LightColor color, LightColor brightnessEndColor, std::uint16_t brightMs, std::uint16_t offMs);
        LightControlData(LightControlMode mode, LightColor color, LightColor brightnessEndColor, std::uint16_t brightMs, std::uint16_t offMs);
    };

    enum RPOS_CORE_API AEBControlCmd : std::uint8_t
    {
        AEBControlCmdOn = 0,
        AEBControlCmdOff = 1,
    };

}}

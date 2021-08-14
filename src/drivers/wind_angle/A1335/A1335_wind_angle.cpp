/****************************************************************************
 *
 *   Copyright (c) 2013, 2014, 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file A1335_wind_angle.cpp
 * @author Teva Zanker
 *
 * Driver for the A1335 series connected via I2C.
 *
 * Supported sensors:
 *
 *    - A1335LLETR (https://www.digikey.ca/en/products/detail/allegro-microsystems/A1335LLETR-DD-T/5774399)
 *
 * Interface application notes:
 *
 *    - Interfacing to A1335 Hall Effect Sensors (https://www.allegromicro.com/-/media/files/programming-manuals/a1335-programming-manual.ashx)
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>


using namespace time_literals;

/* I2C bus address is 0xC */
#define I2C_ADDRESS_A1335    0xC

/* Register address */




class WindAngleA1335 : public device::I2C, public I2CSPIDriver<WindAngleA1335> {
public:
    WindAngleA1335(const I2CSPIDriverConfig &config);

    ~WindAngleA1335() override;

    static void print_usage();

    int init() override;

    void RunImpl();

    void print_status() override;

private:


    enum class Register : uint8_t {
        ANGLE = 0x20
    };

    int _conversion_interval = 31250;

    void start();

    int readRegister(Register reg, uint8_t *data, int len);

    long readAngle();

    int collect();


    long bytesToAngle(uint8_t byte1, uint8_t byte2);

};
/*
 * Driver 'main' command.
 */


WindAngleA1335::WindAngleA1335(const I2CSPIDriverConfig &config) :
        I2C(config),
        I2CSPIDriver(config) {
//TODO I dont think anything needs to be here strictly speaking
}
WindAngleA1335::~WindAngleA1335(){
    //Destructor
}

int WindAngleA1335::init() {
    int ret = I2C::init();

    if (ret == PX4_OK) {
        start();
    }

    return ret;
}

int WindAngleA1335::readRegister(Register reg, uint8_t *data, int len) {
    const uint8_t cmd = (uint8_t)reg;
    return transfer(&cmd, 1, data, len);
    // transfer source: https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/drivers/device/nuttx/I2C.hpp
}

long WindAngleA1335::bytesToAngle(uint8_t byte1, uint8_t byte2) {
    long combined = ((byte1 & 15) << 8) | byte2;
    long angle = combined * 365 / 4096;
    return angle;
}



long WindAngleA1335::readAngle() {
    uint8_t data[2] = {0, 0};
    const int len = 2;
    const uint8_t cmd = (uint8_t)Register::ANGLE;
    transfer(&cmd, 1, data, len);
    long angle = bytesToAngle(data[0], data[1]);
    return angle;
// transfer source: https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/drivers/device/nuttx/I2C.hpp

}

int WindAngleA1335::collect()
{
    long angle = readAngle();
    PX4_INFO("result %ld", angle);
    return 1;
}

void WindAngleA1335::start()
{
    /* schedule a cycle to start things */
    PX4_INFO("Starting Wind Angle");
    ScheduleDelayed(_conversion_interval);
}

void WindAngleA1335::RunImpl() {
    int ret = collect();
    if (OK != ret) {
        DEVICE_DEBUG("measure error");
    }
    ScheduleDelayed(_conversion_interval);
}

void WindAngleA1335::print_status()
{
    I2CSPIDriverBase::print_status();
}

void WindAngleA1335::print_usage() {
    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description

I2C bus driver for A1335 hall effect sensor.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("A1335_wind_angle", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("wind_angle");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}


extern "C" __EXPORT int A1335_wind_angle_main(int argc, char *argv[]) {
    using ThisDriver = WindAngleA1335;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 100000;
    cli.i2c_address = I2C_ADDRESS_A1335;
    //TEst
    cli.getOpt(argc, argv, "");
    const char *verb = cli.optArg();
    PX4_INFO("Reached Main");
    if (!verb) {
        PX4_INFO("No Verb");
        ThisDriver::print_usage();
        return -1;
    }
    PX4_INFO("verb: %s\n", verb);
    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ANGLE_DEVTYPE_A1335);

    if (!strcmp(verb, "start")) {

        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {

        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {

        return ThisDriver::module_status(iterator);
    }

    ThisDriver::print_usage();
    return -1;
}
//C:\PX4\home\PX4-Autopilot\build\px4_fmu-v5_default

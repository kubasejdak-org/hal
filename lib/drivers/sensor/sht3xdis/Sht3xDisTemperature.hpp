////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Jakub Sejdak
/// @copyright TRUMPF Huettinger
///
/// Copyright (c) 2019-2021, TRUMPF Huettinger
/// All rights reserved.
///
////////////////////////////////////////////////////////////////////////

#pragma once

#include "Sht3xDisSensor.h"
#include "hal/sensor/ITemperatureSensor.h"

#include <memory>

namespace tr::hal::sensor {

/**
 * @class Sht3xDisTemperature
 * Represents the SHT3x-DIS driver of the ITemperatureSensor interface.
 */
class Sht3xDisTemperature : public ITemperatureSensor
{
public:
    /**
     * Constructor.
     * @param sensor                Sensor which will provide the measurements.
     */
    explicit Sht3xDisTemperature(std::shared_ptr<Sht3xDisSensor> sensor);

    /**
     * Returns the minimal value of the temperature that can be returned from this device in Celsius degrees.
     * @return Minimal value of the temperature that can be returned from this device in Celsius degrees.
     */
    float minValue() const override { return -40.0F; }

    /**
     * Returns the maximal value of the temperature that can be returned from this device in Celsius degrees.
     * @return Maximal value of the temperature that can be returned from this device in Celsius degrees.
     */
    float maxValue() const override { return 125.0F; }

private:
    /**
     * SHT3x-DIS implementation of the method that reads the current temperature value in Celsius degrees.
     * @param temperature           Output parameter with the temperature value in Celsius degrees.
     * @return Error code of the operation.
     * @retval eOk                  Temperature was successfully read.
     * @retval eNoMem               There is no memory left to allocate space for the data.
     * @retval eError               Driver is not properly initialized.
     * @retval eNotOpened           Device is not opened.
     * @retval eWrongState          Bus is not locked.
     * @retval eTimeout             Timeout occurred before the operation was finished.
     */
    err drvRead(float* temperature) override;

private:
    std::shared_ptr<Sht3xDisSensor> m_sensor;
};

} // namespace tr::hal::sensor

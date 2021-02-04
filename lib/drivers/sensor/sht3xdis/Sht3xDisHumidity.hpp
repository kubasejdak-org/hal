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
#include "hal/sensor/IHumiditySensor.h"

#include <memory>

namespace tr::hal::sensor {

/**
 * @class Sht3xDisHumidity
 * Represents the SHT3x-DIS driver of the IHumiditySensor interface.
 */
class Sht3xDisHumidity : public IHumiditySensor
{
public:
    /**
     * Constructor.
     * @param sensor                Sensor which will provide the measurements.
     */
    explicit Sht3xDisHumidity(std::shared_ptr<Sht3xDisSensor> sensor);

private:
    /**
     * SHT3x-DIS implementation of the method that reads the current relative humidity value.
     * @param relativeHumidity      Output parameter with the relative humidity percentage in range 0-100 %.
     * @return Error code of the operation.
     * @retval eOk                  Humidity was successfully read.
     * @retval eNoMem               There is no memory left to allocate space for the data.
     * @retval eError               Driver is not properly initialized.
     * @retval eNotOpened           Device is not opened.
     * @retval eWrongState          Bus is not locked.
     * @retval eTimeout             Timeout occurred before the operation was finished.
     */
    err drvRead(float* relativeHumidity) override;

private:
    std::shared_ptr<Sht3xDisSensor> m_sensor;
};

} // namespace tr::hal::sensor

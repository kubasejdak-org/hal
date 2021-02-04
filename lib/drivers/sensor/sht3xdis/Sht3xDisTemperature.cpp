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

#include "Sht3xDisTemperature.h"

#include <utility>

namespace tr::hal::sensor {

Sht3xDisTemperature::Sht3xDisTemperature(std::shared_ptr<Sht3xDisSensor> sensor)
    : m_sensor(std::move(sensor))
{}

err Sht3xDisTemperature::drvRead(float* temperature)
{
    Sht3xMeasurement measurement{};
    auto result = m_sensor->getMeasurement(&measurement);
    if (result == err::eOk)
    {
        *temperature = measurement.temperature;
    }

    return result;
}

} // namespace tr::hal::sensor

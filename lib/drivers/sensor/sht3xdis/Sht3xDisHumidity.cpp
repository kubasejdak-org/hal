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

#include "Sht3xDisHumidity.h"

#include <utility>

namespace tr::hal::sensor {

Sht3xDisHumidity::Sht3xDisHumidity(std::shared_ptr<Sht3xDisSensor> sensor)
    : m_sensor(std::move(sensor))
{}

err Sht3xDisHumidity::drvRead(float* relativeHumidity)
{
    Sht3xMeasurement measurement{};
    auto result = m_sensor->getMeasurement(&measurement);
    if (result == err::eOk) {
        *relativeHumidity = measurement.relativeHumidity;
    }

    return result;
}

} // namespace tr::hal::sensor

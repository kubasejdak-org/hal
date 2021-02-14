/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2021-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// 1. Redistributions of source code must retain the above copyright notice, this
///    list of conditions and the following disclaimer.
///
/// 2. Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
/// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
/// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
/// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
/// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
/// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
/////////////////////////////////////////////////////////////////////////////////////

#include <hal/Error.hpp>
#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/sensor/IHumiditySensor.hpp>
#include <hal/sensor/ITemperatureSensor.hpp>

#include <catch2/catch.hpp>

#include <cstdio>

TEST_CASE("1. Check if humidity values make sense", "[unit][sensor]")
{
    hal::ScopedHardware hardware;

    auto humidity = hal::getScopedDevice<hal::sensor::IHumiditySensor>(hal::device_id::eSht3xDisHumidity);

    float relativeHumidity{};
    auto error = humidity->read(relativeHumidity);
    REQUIRE(!error);
    REQUIRE(humidity->minValue() >= 0.0F);
    REQUIRE(humidity->maxValue() <= 100.0F);
    REQUIRE(relativeHumidity >= humidity->minValue());
    REQUIRE(relativeHumidity <= humidity->maxValue());

    std::printf("Humidity: %2.2f %%\n", relativeHumidity);

    // These values should be valid for tests performed in typical home environment.
    constexpr auto cMinHumidity = 10.0F;
    constexpr auto cMaxHumidity = 80.0F;
    REQUIRE(relativeHumidity >= cMinHumidity);
    REQUIRE(relativeHumidity <= cMaxHumidity);
}

TEST_CASE("2. Check if temperature values make sense", "[unit][sensor]")
{
    hal::ScopedHardware hardware;

    auto temperature = hal::getScopedDevice<hal::sensor::ITemperatureSensor>(hal::device_id::eSht3xDisTemperature);

    float temperatureValue{};
    auto error = temperature->read(temperatureValue);
    REQUIRE(!error);
    REQUIRE(temperatureValue >= temperature->minValue());
    REQUIRE(temperatureValue <= temperature->maxValue());

    std::printf("Temperature: %2.2f °C\n", temperatureValue);

    // These values should be valid for tests performed in typical home environment.
    constexpr auto cMinTemperature = 10.0F;
    constexpr auto cMaxTemperature = 35.0F;
    REQUIRE(temperatureValue >= cMinTemperature);
    REQUIRE(temperatureValue <= cMaxTemperature);
}

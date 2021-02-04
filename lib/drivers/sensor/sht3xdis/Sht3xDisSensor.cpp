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

#include "Sht3xDisSensor.hpp"

#include "hal/i2c/ScopedI2c.hpp"
#include "hal/utils/logger.hpp"

#include <osal/sleep.hpp>

#include <cassert>
#include <cmath>
#include <utility>

namespace hal::sensor {

/// Default timeout for SHT3x read operation.
constexpr std::uint32_t cSht3xReadTimeoutMs = 15;

/// SHT3x command for single shot read with clock stretching disabled and high repeatability.
constexpr std::uint16_t cSht3CmdReadCsd = 0x0024;

/// Converts raw temperature measurement to the physical temperature in Celsius degrees.
/// @param rawTemperature       Raw temperature to be converted.
/// @return Physical temperature in Celsius degrees.
static inline float rawTemperatureToPhysical(std::uint16_t rawTemperature)
{
    return (-45.0F + (175.0F * static_cast<float>(rawTemperature)) / (std::pow(2.0F, 16.0F) - 1.0F)); // NOLINT
}

/// Converts raw humidity measurement to the physical relative humidity in percentage.
/// @param rawHumidity          Raw humidity to be converted
/// @return Relative humidity in percentage.
static inline float rawHumidityToPhysical(std::uint16_t rawHumidity)
{
    return (static_cast<float>(rawHumidity) * 100.0F) / (std::pow(2.0F, 16.0F) - 1.0F); // NOLINT
}

Sht3xDisSensor::Sht3xDisSensor(std::shared_ptr<i2c::II2c> i2c,
                               i2c::AddressingMode addressingMode,
                               std::uint16_t address,
                               std::chrono::milliseconds refreshThreshold)
    : m_i2c(std::move(i2c))
    , m_addressingMode(addressingMode)
    , m_address(address)
    , m_refreshThreshold(refreshThreshold)
{
    if (!i2c::verifyAddress(m_addressingMode, m_address)) {
        Sht3xLogger::critical("Failed to create SHT3x sensor: bad parameters");
        assert(false);
        return;
    }

    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        Sht3xLogger::critical("Failed to create SHT3x sensor: timeout when locking I2C bus (timeout=default)");
        assert(false);
        return;
    }

    m_i2c->open();

    Sht3xLogger::info("Created SHT3x-DIS sensor with the following parameters:");
    Sht3xLogger::info("  addressingMode     : {}", (addressingMode == i2c::AddressingMode::e7bit) ? "7-bit" : "10-bit");
    Sht3xLogger::info("  address            : {:#x}", address);
    Sht3xLogger::info("  refreshThresholdMs : {}", m_refreshThreshold);
}

Sht3xDisSensor::~Sht3xDisSensor()
{
    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        Sht3xLogger::warn("Failed to close I2C device: timeout when locking I2C bus (timeout=default)");
        return;
    }

    m_i2c->close();
}

std::error_code Sht3xDisSensor::getMeasurement(Sht3xMeasurement& measurement)
{
    if (measurement == nullptr) {
        Sht3xLogger::error("Failed to get measurement: measurement=nullptr");
        return err::eInvalidArgument;
    }

    measurement->temperature = 0.0F;
    measurement->relativeHumidity = 0.0F;

    trOsalMutexLock(&m_mutex);

    if ((trOsalTimestampNowUs() - m_lastReadTimestampUs) <= m_refreshThresholdUs) {
        Sht3xLogger::trace("Cache didn't expire, using old value");
        *measurement = m_measurement;
        trOsalMutexUnlock(&m_mutex);
        return err::eOk;
    }

    i2c::ScopedI2c lock(m_i2c);
    if (!lock.isAcquired()) {
        Sht3xLogger::warn("Failed to get measurement: timeout when locking I2C bus (timeout=default)");
        trOsalMutexUnlock(&m_mutex);
        return err::eTimeout;
    }

    err result = m_i2c->write(m_address,
                              reinterpret_cast<const std::uint8_t*>(&cSht3CmdReadCsd),
                              sizeof(cSht3CmdReadCsd),
                              true,
                              cTimeoutInfinite);
    if (result == err::eOk) {
        const int cTrials = 2;
        for (int i = 0; i < cTrials; ++i) {
            trOsalSleepMs(cSht3xReadTimeoutMs);

            std::array<std::uint8_t, 6> bytes{};
            std::size_t actualReadSize;
            result = m_i2c->read(m_address, bytes.data(), bytes.size(), cTimeoutInfinite, &actualReadSize);
            if (result == err::eOk && actualReadSize == bytes.size()) {
                std::uint16_t rawTemperature
                    = ((static_cast<std::uint16_t>(bytes[0]) << 8) | static_cast<std::uint16_t>(bytes[1]));
                std::uint16_t rawHumidity
                    = ((static_cast<std::uint16_t>(bytes[3]) << 8) | static_cast<std::uint16_t>(bytes[4]));

                m_measurement.temperature = rawTemperatureToPhysical(rawTemperature);
                m_measurement.relativeHumidity = rawHumidityToPhysical(rawHumidity);
                m_lastReadTimestampUs = trOsalTimestampNowUs();

                *measurement = m_measurement;
                break;
            }

            Sht3xLogger::error("Failed to get measurement: I2C read returned err={}, actualReadSize={}",
                               toString(result),
                               actualReadSize);
        }
    }
    else {
        Sht3xLogger::error("Failed to get measurement: I2C write returned err={}", toString(result));
    }

    lock.release();
    trOsalMutexUnlock(&m_mutex);

    return result;
}

} // namespace hal::sensor

/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2021, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#pragma once

#include <algorithm>
#include <memory>
#include <type_traits>

namespace hal {

class Device;

template <typename T>
class ScopedDevice {
    static_assert(std::is_base_of_v<Device, T>, "ScopedDevice can hold only types derived from hal::Device");

public:
    /// Default constructor.
    ScopedDevice() = default;

    ScopedDevice(std::shared_ptr<T> device) // NOLINT
        : m_device(std::move(device))
    {}
    ScopedDevice(const ScopedDevice&) = delete;
    ScopedDevice(ScopedDevice&& other) noexcept = default;
    ~ScopedDevice()
    {
        if (m_device) {
            returnDevice(m_device);
            m_device.reset();
        }
    }
    ScopedDevice<T>& operator=(const ScopedDevice&) = delete;
    ScopedDevice<T>& operator=(ScopedDevice&&) noexcept = default;

    operator std::shared_ptr<T>() const { return m_device; }      // NOLINT
    operator bool() const { return static_cast<bool>(m_device); } // NOLINT

    auto operator->() const { return m_device; }
    auto& operator*() const { return *m_device; }

    void reset() { m_device.reset(); }
    void swap(ScopedDevice<T>& other) { std::swap(other.m_device, m_device); }
    T* get() const { return m_device.get(); }

private:
    std::shared_ptr<T> m_device;
};

} // namespace hal

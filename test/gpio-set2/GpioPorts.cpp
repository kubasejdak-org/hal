/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019-2020, Kuba Sejdak <kuba.sejdak@gmail.com>
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

#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/gpio/IPortInput.hpp>
#include <hal/gpio/IPortOutput.hpp>

#include <catch2/catch.hpp>

#include <cstdint>

TEST_CASE("Toggle values of single pins", "[unit][gpio]")
{
    hal::Hardware::init();
    hal::Hardware::attach();

    std::vector<std::shared_ptr<hal::gpio::IPortOutput<std::uint8_t>>> outputs;
    std::vector<std::shared_ptr<hal::gpio::IPortInput<std::uint8_t>>> inputs;

    // clang-format off
    outputs.emplace_back(hal::getDevice<hal::gpio::IPortOutput<std::uint8_t>>(hal::device_id::GpioSet2Id::ePortAPinSet0));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPortOutput<std::uint8_t>>(hal::device_id::GpioSet2Id::ePortAPinSet1));

    inputs.emplace_back(hal::getDevice<hal::gpio::IPortInput<std::uint8_t>>(hal::device_id::GpioSet2Id::ePortBPinSet0));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPortInput<std::uint8_t>>(hal::device_id::GpioSet2Id::ePortBPinSet1));
    // clang-format on

    for (auto& output : outputs)
        hal::returnDevice(output);

    for (auto& input : inputs)
        hal::returnDevice(input);

    hal::Hardware::detach();
}

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

#include <hal/Error.hpp>
#include <hal/Hardware.hpp>
#include <hal/factory.hpp>
#include <hal/gpio/IPinInput.hpp>
#include <hal/gpio/IPinOutput.hpp>

#include <catch2/catch.hpp>

TEST_CASE("Toggle values of single pins", "[unit][gpio]")
{
    hal::Hardware::init();
    hal::Hardware::attach();

    std::vector<std::shared_ptr<hal::gpio::IPinOutput>> outputs;
    std::vector<std::shared_ptr<hal::gpio::IPinInput>> inputs;

    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin0));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin1));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin2));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin3));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin4));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin5));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin6));
    outputs.emplace_back(hal::getDevice<hal::gpio::IPinOutput>(hal::device_id::GpioSet1Id::ePortAPin7));

    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin0));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin1));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin2));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin3));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin4));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin5));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin6));
    inputs.emplace_back(hal::getDevice<hal::gpio::IPinInput>(hal::device_id::GpioSet1Id::ePortBPin7));

    SECTION("Multiple toggling")
    {
        bool setValue = true;
        constexpr int cTestIterations = 100;
        for (int i = 0; i < cTestIterations; ++i) {
            for (auto& output : outputs) {
                auto result = output->set(setValue);
                REQUIRE(result == hal::Error::eOk);
            }

            for (auto& input : inputs) {
                bool getValue{};
                auto result = input->get(getValue);
                REQUIRE(result == hal::Error::eOk);
                REQUIRE(getValue == setValue);
            }

            setValue = !setValue;
        }
    }

    SECTION("Set first and last bit outputs to low, check all")
    {
        // Set all outputs to high.
        for (auto& output : outputs) {
            auto result = output->on();
            REQUIRE(result == hal::Error::eOk);
        }

        // Verify initial setup.
        for (auto& input : inputs) {
            bool getValue{};
            auto result = input->get(getValue);
            REQUIRE(result == hal::Error::eOk);
            REQUIRE(getValue);
        }

        // Set first and last bit outputs to low.
        outputs.front()->off();
        outputs.back()->off();

        // Verify test setup.
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            bool getValue{};
            auto result = inputs[i]->get(getValue);
            REQUIRE(result == hal::Error::eOk);
            bool shouldBeLow = (i == 0 || i == (inputs.size() - 1));
            REQUIRE(getValue == !shouldBeLow);
        }
    }

    SECTION("Toggle only one pin at a time")
    {
        for (std::size_t i = 0; i < outputs.size(); ++i) {
            for (std::size_t j = 0; j < outputs.size(); ++j) {
                bool value = (i == j);
                auto result = outputs[j]->set(value);
                REQUIRE(result == hal::Error::eOk);
            }

            for (std::size_t j = 0; j < outputs.size(); ++j) {
                bool getValue{};
                auto result = inputs[j]->get(getValue);
                REQUIRE(result == hal::Error::eOk);

                bool expectedValue = (i == j);
                REQUIRE(expectedValue == getValue);
            }
        }
    }

    for (auto& output : outputs)
        hal::returnDevice(output);

    for (auto& input : inputs)
        hal::returnDevice(input);

    hal::Hardware::detach();
}

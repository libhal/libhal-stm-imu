// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
namespace hal::stm_imu {
struct hardware_map
{
  hal::serial* console;
  hal::i2c* i2c;
  hal::spi* spi;
  hal::output_pin* output_pin;
  hal::steady_clock* clock;
  hal::callback<void()> reset;
};
}  // namespace hal::stm_imu
// Application function must be implemented by one of the compilation units
// (.cpp) files.
hal::status application(hal::stm_imu::hardware_map& p_map);
hal::result<hal::stm_imu::hardware_map> initialize_platform();

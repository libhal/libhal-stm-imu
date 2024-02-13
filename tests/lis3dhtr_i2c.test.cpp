// Copyright 2023 Google LLC
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
#include <boost/ut.hpp>
<<<<<<<< HEAD:tests/lis3dhtr_i2c.test.cpp
#include <libhal-stm-imu/lis3dhtr_i2c.hpp>

namespace hal::stm_imu {
void lis3dhtr_i2c_test()
========
#include <libhal-stm-imu/lis3dhtr.hpp>

namespace hal::stm_imu {
void lis3dhtr_test()
>>>>>>>> 787b047 (:sparkles: Add basic working support for stm's lis3dhtr imu, I2C):tests/lis3dhtr.test.cpp
{
  using namespace boost::ut;
  using namespace std::literals;

<<<<<<<< HEAD:tests/lis3dhtr_i2c.test.cpp
  "lis3dhtr_i2c::create()"_test = []() {
========
  "lis3dhtr::create()"_test = []() {
>>>>>>>> 787b047 (:sparkles: Add basic working support for stm's lis3dhtr imu, I2C):tests/lis3dhtr.test.cpp
    // Setup
    // Exercise
    // Verify
  };
};
}  // namespace hal::stm_imu
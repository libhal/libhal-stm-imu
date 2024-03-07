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

#include "../include/libhal-stm-imu/lis3dhtr_i2c.hpp"
#include "lis3dhtr_constants.hpp"

namespace hal::stm_imu {
using namespace std::chrono_literals;
using namespace hal::literals;

// public
result<lis3dhtr_i2c> lis3dhtr_i2c::create(hal::i2c& p_i2c,
                                          hal::byte p_device_address,
                                          max_acceleration p_gscale)
{
  lis3dhtr_i2c lis(p_i2c, p_device_address, p_gscale);
  HAL_CHECK(lis.verify_device());
  HAL_CHECK(lis.power_on());
  HAL_CHECK(lis.configure_full_scale(p_gscale));
  return lis;
}

hal::status lis3dhtr_i2c::verify_device()
{
  // the expected value as read from the data sheet is 0x33
  constexpr auto expected = 0x33;
  auto who_am_i = HAL_CHECK(
    hal::write_then_read<1>(*m_i2c,
                            m_address,
                            std::array{ hal::stm_imu::who_am_i_register },
                            hal::never_timeout()));

  if (who_am_i[0] != expected) {
    return hal::new_error();
  }

  return hal::success();
}

hal::status lis3dhtr_i2c::power_on()
{
  HAL_CHECK(configure_data_rates(data_rate_config::mode_7));
  return hal::success();
}

hal::status lis3dhtr_i2c::power_off()
{
  HAL_CHECK(configure_data_rates(data_rate_config::mode_0));
  return hal::success();
}

hal::status lis3dhtr_i2c::configure_data_rates(data_rate_config p_data_rate)
{

  constexpr auto configure_reg_bit_mask = hal::bit_mask::from<7, 4>();
  auto ctrl_reg1_array = std::array{ ctrl_reg1 };
  auto ctrl_reg1_data = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c, m_address, ctrl_reg1_array, hal::never_timeout()));

  hal::bit_modify(ctrl_reg1_data[0])
    .insert<configure_reg_bit_mask>(static_cast<hal::byte>(p_data_rate));

  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array{ ctrl_reg1, ctrl_reg1_data[0] },
                       hal::never_timeout()));
  return hal::success();
}

hal::status lis3dhtr_i2c::configure_full_scale(max_acceleration p_gravity_code)
{

  m_gscale = static_cast<hal::byte>(p_gravity_code);

  constexpr auto configure_reg_bit_mask = hal::bit_mask::from<5, 4>();
  auto ctrl_reg4_array = std::array{ ctrl_reg4 };
  auto ctrl_reg4_data = HAL_CHECK(hal::write_then_read<1>(
    *m_i2c, m_address, ctrl_reg4_array, hal::never_timeout()));
  hal::bit_modify(ctrl_reg4_data[0])
    .insert<configure_reg_bit_mask>(static_cast<hal::byte>(p_gravity_code));

  HAL_CHECK(hal::write(*m_i2c,
                       m_address,
                       std::array{ ctrl_reg4, ctrl_reg4_data[0] },
                       hal::never_timeout()));
  return hal::success();
}

// private

lis3dhtr_i2c::lis3dhtr_i2c(hal::i2c& p_i2c,
                           hal::byte p_device_address,
                           max_acceleration p_gscale)
  : m_i2c(&p_i2c)
  , m_address(p_device_address)
  , m_gscale(static_cast<hal::byte>(p_gscale))
{
}

hal::result<accelerometer::read_t> lis3dhtr_i2c::driver_read()
{
  accelerometer::read_t acceleration;
  constexpr auto number_of_axis = 3;
  constexpr auto bytes_per_axis = 2;
  auto xyz_acceleration =
    HAL_CHECK(hal::write_then_read<number_of_axis * bytes_per_axis>(
      *m_i2c,
      m_address,
      std::array{ hal::stm_imu::read_xyz_axis },
      hal::never_timeout()));

  /* parsing data from accelerometer
   all data is left justified which means the data will always have the lowest
   nibble be 0 so we shift it all to the right by 4 and or the low and high
   bytes together 0000'0000'0000'0000
  */
  constexpr auto read_h_bit_mask = hal::bit_mask::from<15, 8>();

  auto x = static_cast<uint16_t>(xyz_acceleration[0]);
  hal::bit_modify(x).insert<read_h_bit_mask>(
    static_cast<uint16_t>(xyz_acceleration[1]));

  auto y = static_cast<uint16_t>(xyz_acceleration[2]);
  hal::bit_modify(y).insert<read_h_bit_mask>(
    static_cast<uint16_t>(xyz_acceleration[3]));

  auto z = static_cast<uint16_t>(xyz_acceleration[4]);
  hal::bit_modify(z).insert<read_h_bit_mask>(
    static_cast<uint16_t>(xyz_acceleration[5]));

  // output_limits is affected by the configured g_scale, if it is set to 2 then
  // the code is 0, add one to the code and shift 1 by that many, this will
  // succesfully allow you to map the output to the different amount g's that
  // are selected.
  auto output_limits =
    static_cast<float>(1 << (static_cast<int16_t>(m_gscale) + 1));

  constexpr auto max = static_cast<float>(std::numeric_limits<int16_t>::max());
  constexpr auto min = static_cast<float>(std::numeric_limits<int16_t>::min());

  auto input_range = std::make_pair(max, min);
  auto output_range = std::make_pair(-output_limits, output_limits);

  acceleration.x = hal::map(static_cast<int16_t>(x), input_range, output_range);
  acceleration.y = hal::map(static_cast<int16_t>(y), input_range, output_range);
  acceleration.z = hal::map(static_cast<int16_t>(z), input_range, output_range);

  return acceleration;
}

}  // namespace hal::stm_imu

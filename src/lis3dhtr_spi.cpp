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

#include "../include/libhal-stm-imu/lis3dhtr_spi.hpp"
#include "lis3dhtr_constants.hpp"
#include <libhal-util/steady_clock.hpp>
#include <libhal/serial.hpp>

namespace hal::stm_imu {
using namespace std::chrono_literals;
using namespace hal::literals;

// public
result<lis3dhtr_spi> lis3dhtr_spi::create(hal::spi& p_spi,
                                          hal::output_pin& p_cs,
                                          max_acceleration p_gscale)
{
  lis3dhtr_spi lis(p_spi, p_cs, p_gscale);
  HAL_CHECK(lis.configure_spi_mode(spi_mode::four_wire));
  HAL_CHECK(lis.verify_device());
  HAL_CHECK(lis.power_on());
  HAL_CHECK(lis.configure_full_scale(p_gscale));
  return lis;
}

hal::status lis3dhtr_spi::verify_device()
{
  // the expected value as read from the data sheet is 0x33
  constexpr auto expected = 0x33;
  constexpr auto addr_bit_mask = hal::bit_mask::from<5, 0>();

  auto read_from_who_am_i = hal::bit_value(0U)
                              .insert<addr_bit_mask>(who_am_i_register)
                              .set<spi_read_bit_mask>()
                              .to<hal::byte>();

  HAL_CHECK(m_cs->level(false));
  auto who_am_i = HAL_CHECK(
    hal::write_then_read<1>(*m_spi, std::array{ read_from_who_am_i }));
  HAL_CHECK(m_cs->level(true));
  // TODO(#<5>): This error needs to be more specific
  if (who_am_i[0] != expected) {
    return hal::new_error();
  }

  return hal::success();
}

hal::status lis3dhtr_spi::power_on()
{
  HAL_CHECK(configure_data_rates(data_rate_config::mode_7));
  return hal::success();
}

hal::status lis3dhtr_spi::power_off()
{
  HAL_CHECK(configure_data_rates(data_rate_config::mode_0));
  return hal::success();
}

hal::status lis3dhtr_spi::configure_data_rates(data_rate_config p_data_rate)
{
  constexpr auto configure_reg_bit_mask = hal::bit_mask::from<7, 4>();
  constexpr auto addr_bit_mask = hal::bit_mask::from<5, 0>();

  auto read_from_ctrl_reg1 = hal::bit_value(0U)
                               .insert<addr_bit_mask>(ctrl_reg1)
                               .set<spi_read_bit_mask>()
                               .to<hal::byte>();
  auto write_to_ctrl_reg1 = hal::bit_value(0U)
                              .insert<addr_bit_mask>(ctrl_reg1)
                              .set<spi_addr_inc_bit_mask>()
                              .to<hal::byte>();

  HAL_CHECK(m_cs->level(false));
  auto ctrl_reg1_data = HAL_CHECK(
    hal::write_then_read<1>(*m_spi, std::array{ read_from_ctrl_reg1 }));
  HAL_CHECK(m_cs->level(true));

  hal::bit_modify(ctrl_reg1_data[0])
    .insert<configure_reg_bit_mask>(static_cast<hal::byte>(p_data_rate));

  HAL_CHECK(m_cs->level(false));
  HAL_CHECK(
    hal::write(*m_spi, std::array{ write_to_ctrl_reg1, ctrl_reg1_data[0] }));
  HAL_CHECK(m_cs->level(true));

  return hal::success();
}

hal::status lis3dhtr_spi::configure_full_scale(max_acceleration p_gravity_code)
{
  m_gscale = static_cast<hal::byte>(p_gravity_code);

  constexpr auto configure_reg_bit_mask = hal::bit_mask::from<5, 4>();
  constexpr auto addr_bit_mask = hal::bit_mask::from<5, 0>();

  auto read_from_ctrl_reg4 = hal::bit_value(0U)
                               .insert<addr_bit_mask>(ctrl_reg4)
                               .set<spi_read_bit_mask>()
                               .to<hal::byte>();
  auto write_to_ctrl_reg4 = hal::bit_value(0U)
                              .insert<addr_bit_mask>(ctrl_reg4)
                              .set<spi_addr_inc_bit_mask>()
                              .to<hal::byte>();

  auto ctrl_reg4_array = std::array{ read_from_ctrl_reg4 };

  HAL_CHECK(m_cs->level(false));
  auto ctrl_reg4_data =
    HAL_CHECK(hal::write_then_read<1>(*m_spi, ctrl_reg4_array));
  HAL_CHECK(m_cs->level(true));

  hal::bit_modify(ctrl_reg4_data[0])
    .insert<configure_reg_bit_mask>(static_cast<hal::byte>(p_gravity_code));

  HAL_CHECK(m_cs->level(false));
  HAL_CHECK(
    hal::write(*m_spi, std::array{ write_to_ctrl_reg4, ctrl_reg4_data[0] }));
  HAL_CHECK(m_cs->level(true));

  return hal::success();
}

// private

lis3dhtr_spi::lis3dhtr_spi(hal::spi& p_spi,
                           hal::output_pin& p_cs,
                           max_acceleration p_gscale)
  : m_spi(&p_spi)
  , m_cs(&p_cs)
  , m_gscale(static_cast<hal::byte>(p_gscale))
{
}

hal::result<accelerometer::read_t> lis3dhtr_spi::driver_read()
{
  accelerometer::read_t acceleration;
  constexpr auto number_of_axis = 3;
  constexpr auto bytes_per_axis = 2;
  constexpr auto addr_bit_mask = hal::bit_mask::from<5, 0>();

  auto read_from_xyz_register = hal::bit_value(0U)
                                  .insert<addr_bit_mask>(out_x_l)
                                  .set<spi_read_bit_mask>()
                                  .set<spi_addr_inc_bit_mask>()
                                  .to<hal::byte>();

  HAL_CHECK(m_cs->level(false));
  auto xyz_acceleration =
    HAL_CHECK(hal::write_then_read<number_of_axis * bytes_per_axis>(
      *m_spi, std::array{ read_from_xyz_register }));
  HAL_CHECK(m_cs->level(true));

  /* parsing data from accelerometer
   all data is left justified which means the data will always have the lowest
   nibble be 0 so we shift it all to the right by 4 and or the low and high
   bytes together 0000'0000'0000'0000
  */
  constexpr auto read_l_bit_mask = hal::bit_mask::from<7, 0>();
  constexpr auto read_h_bit_mask = hal::bit_mask::from<15, 8>();

  auto x_l_acceleration = static_cast<uint16_t>(xyz_acceleration[0]);
  auto x_h_acceleration = static_cast<uint16_t>(xyz_acceleration[1]);
  auto y_l_acceleration = static_cast<uint16_t>(xyz_acceleration[2]);
  auto y_h_acceleration = static_cast<uint16_t>(xyz_acceleration[3]);
  auto z_l_acceleration = static_cast<uint16_t>(xyz_acceleration[4]);
  auto z_h_acceleration = static_cast<uint16_t>(xyz_acceleration[5]);

  auto x = hal::bit_value(0U)
             .insert<read_l_bit_mask>(x_l_acceleration)
             .insert<read_h_bit_mask>(x_h_acceleration)
             .to<uint16_t>();

  auto y = hal::bit_value(0U)
             .insert<read_l_bit_mask>(y_l_acceleration)
             .insert<read_h_bit_mask>(y_h_acceleration)
             .to<uint16_t>();

  auto z = hal::bit_value(0U)
             .insert<read_l_bit_mask>(z_l_acceleration)
             .insert<read_h_bit_mask>(z_h_acceleration)
             .to<uint16_t>();

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

hal::status lis3dhtr_spi::configure_spi_mode(spi_mode p_spi_mode)
{

  constexpr auto configure_reg_bit_mask = hal::bit_mask::from<0>();
  constexpr auto addr_bit_mask = hal::bit_mask::from<5, 0>();

  auto read_from_ctrl_reg4 = hal::bit_value(0U)
                               .set<spi_read_bit_mask>()
                               .insert<addr_bit_mask>(ctrl_reg4)
                               .to<hal::byte>();
  auto write_to_ctrl_reg4 = hal::bit_value(0U)
                              .set<spi_addr_inc_bit_mask>()
                              .insert<addr_bit_mask>(ctrl_reg4)
                              .to<hal::byte>();

  auto ctrl_reg4_array = std::array{ read_from_ctrl_reg4 };

  HAL_CHECK(m_cs->level(false));
  auto ctrl_reg4_data =
    HAL_CHECK(hal::write_then_read<1>(*m_spi, ctrl_reg4_array));
  HAL_CHECK(m_cs->level(true));
  hal::bit_modify(ctrl_reg4_data[0])
    .insert<configure_reg_bit_mask>(static_cast<hal::byte>(p_spi_mode));
  HAL_CHECK(m_cs->level(false));
  HAL_CHECK(
    hal::write(*m_spi, std::array{ write_to_ctrl_reg4, ctrl_reg4_data[0] }));
  HAL_CHECK(m_cs->level(true));

  return hal::success();
}

}  // namespace hal::stm_imu

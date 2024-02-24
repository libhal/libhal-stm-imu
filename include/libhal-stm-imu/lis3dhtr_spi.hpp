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

#include <libhal-util/bit.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/spi.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>

namespace hal::stm_imu {
class lis3dhtr_spi : public hal::accelerometer
{
public:
  /// @brief max_acceleration is the maxium g's that the device will read
  /// NOTE: the higher the max gravity you select, the lower your resolution is
  enum class max_acceleration : hal::byte
  {
    /// 2x the average earth gravity, acceleration
    g2 = 0x00,
    /// 4x the average earth gravity, acceleration
    g4 = 0x01,
    /// 8x the average earth gravity, acceleration
    g8 = 0x02,
    /// 16x the average earth gravity, acceleration
    g16 = 0x03,
  };

  /// @brief data_rate_config are the different data rates that the imu can be
  /// programmed to output data at in the different modes
  enum class data_rate_config : hal::byte
  {
    // the following set all resolution modes to the same data rates listed
    // 0Hz (this is the power down command)
    mode_0 = 0b0000,
    // 1Hz
    mode_1 = 0b0001,
    // 10Hz
    mode_2 = 0b0010,
    // 25Hz
    mode_3 = 0b0011,
    // 50Hz
    mode_4 = 0b0100,
    // 100Hz
    mode_5 = 0b0101,
    // 200Hz
    mode_6 = 0b0110,
    // 400Hz
    mode_7 = 0b0111,
    // just low power mode is configured in this one to 1.6kHz
    // this is also the default mode set by power_on
    mode_8 = 0b1000,
    // High resolution = normal = 1.344kHz; low power mode = 5.376kHz
    mode_9 = 0b1001,
  };

  /// @brief spi_mode are the two different spi modes that the device supports
  enum class spi_mode : hal::byte
  {
    // this enabled 4 wire spi mode (full duplex)
    four_wire = 0b0,
    // this enabled 3 wire spi mode (half duplex)
    three_wire = 0b1,
  };

  /**
   * @brief Constructs and returns lis object
   *
   * @param p_spi - spi bus the lis is connected to
   * @param p_gscale - The full scale setting for the imu, defaults to 2g
   * @return lis3dhtr_spi object
   * @throws std::errc::protocol_error - when ID register does not match
   * the expected ID for the lis3dhtr_spi device.
   */
  static result<lis3dhtr_spi> create(
    hal::spi& p_spi,
    hal::output_pin& p_cs,
    max_acceleration p_gscale = max_acceleration::g2);

  /**
   * @brief verify's that the device exists on the spi line
   *
   * @return hal::status - success or errors from spi communication
   */
  [[nodiscard]] hal::status verify_device();

  /**
   * @brief Re-enables acceleration readings from the lis
   *
   * @return hal::status - success or errors from spi communication
   * @throws std::errc::invalid_byte_sequence - when ID register does not match
   * the expected ID for the lis3dhtr_spi device.
   */
  [[nodiscard]] hal::status power_on();

  /**
   * @brief Disables acceleration reading from the lis3dhtr_spi.
   *
   * @return hal::status - success or errors from spi communication
   */
  [[nodiscard]] hal::status power_off();

  /**
   * @brief Configures the frequency that new data can be read from the device
   *
   * @param p_data_rate - the frequency that new data can be read from the
   * device
   * @return hal::status - success or errors from spi communication
   */
  [[nodiscard]] hal::status configure_data_rates(data_rate_config p_data_rate);

  /**
   * @brief Changes the gravity scale that the lis is reading. The larger the
   * scale, the less precise the reading.
   *
   * @param p_gravity_code - Scales in powers of 2 up to 16.
   * @return hal::status - success or errors from spi communication
   */
  [[nodiscard]] hal::status configure_full_scale(
    max_acceleration p_gravity_code);

private:
  /**
   * @brief lis3dhtr_spi Constructor
   *
   * @param p_spi - spi peripheral used to commnicate with device.
   * @param p_address - lis3dhtr_spi device address.
   */
  lis3dhtr_spi(spi& p_spi, hal::output_pin& p_cs, max_acceleration p_gscale);

  hal::result<accelerometer::read_t> driver_read();

  /**
   * @brief Changes what spi wire mode that the device will use to transfer data
   *
   * @param p_spi_mode - The spi mode that the device will use
   * @return hal::status - success or errors from spi communication
   */
  [[nodiscard]] hal::status configure_spi_mode(spi_mode p_spi_mode);

  /// The spi peripheral used for communication with the device.
  hal::spi* m_spi;

  // The output pin used to select the lis3dhtr on the spi bus
  hal::output_pin* m_cs;

  /// The minimum and maxium g's that the device will read
  hal::byte m_gscale;
};
}  // namespace hal::stm_imu

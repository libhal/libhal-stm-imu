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
  /**
   * @brief max_acceleration is the maxium g's that the device will read
   * NOTE: the higher the max gravity you select, the lower your resolution is
   */
  enum class max_acceleration : hal::byte
  {
    /**
     * @brief 2x the average earth gravity, acceleration
     */
    g2 = 0x00,
    /**
     * @brief 4x the average earth gravity, acceleration
     */
    g4 = 0x01,
    /**
     * @brief 8x the average earth gravity, acceleration
     */
    g8 = 0x02,
    /**
     * @brief 16x the average earth gravity, acceleration
     */
    g16 = 0x03,
  };

  /**
   * @brief data_rate_config are the different data rates that the imu can be
   * programmed to output data at in the different modes
   */
  enum class data_rate_config : hal::byte
  {
    /**
     * @brief 0Hz (this is the power down command)
     */
    mode_0 = 0b0000,
    /**
     * @brief 1Hz
     */
    mode_1 = 0b0001,
    /**
     * @brief 10Hz
     */
    mode_2 = 0b0010,
    /**
     * @brief 25Hz
     */
    mode_3 = 0b0011,
    /**
     * @brief 50Hz
     */
    mode_4 = 0b0100,
    /**
     * @brief 100Hz
     */
    mode_5 = 0b0101,
    /**
     * @brief 200Hz
     */
    mode_6 = 0b0110,
    /**
     * @brief 400Hz
     */
    mode_7 = 0b0111,
    /**
     * @brief just low power mode is configured in this one to 1.6kHz
     * this is also the default mode set by power_on
     */
    mode_8 = 0b1000,
    /**
     * @brief High resolution = normal = 1.344kHz; low power mode = 5.376kHz
     */
    mode_9 = 0b1001,
  };

  /**
   * @brief spi_mode are the two different spi modes that the device supports
   */
  enum class spi_mode : hal::byte
  {
    /**
     * @brief this enabled 4 wire spi mode (full duplex)
     */
    four_wire = 0b0,
    /**
     * @brief this enabled 3 wire spi mode (half duplex)
     */
    three_wire = 0b1,
  };

  /**
   * @brief Constructs lis object
   *
   * @param p_spi - spi bus the lis is connected to
   * @param p_cs - The chip select to choose this chip to read and write to
   * @param p_gscale - The full scale setting for the imu, defaults to 2g
   *
   * @throws hal::no_such_device - when ID register does not match
   * the expected ID for the lis3dhtr_spi device.
   */
  lis3dhtr_spi(spi& p_spi,
               hal::output_pin& p_cs,
               max_acceleration p_gscale = max_acceleration::g2);

  /**
   * @brief verify's that the device exists on the spi line
   *
   * @throws hal::no_such_device - when ID register does not match
   * the expected ID for the lis3dhtr_spi device.
   */
  void verify_device();

  /**
   * @brief enables acceleration readings from the lis
   *
   */
  void power_on();

  /**
   * @brief Disables acceleration reading from the lis3dhtr_spi.
   *
   */
  void power_off();

  /**
   * @brief Configures the frequency that new data can be read from the device
   *
   * @param p_data_rate - the frequency that new data can be read from the
   * device
   */
  void configure_data_rates(data_rate_config p_data_rate);

  /**
   * @brief Changes the gravity scale that the lis is reading. The larger the
   * scale, the less precise the reading.
   *
   * @param p_gravity_code - Scales in powers of 2 up to 16.
   */
  void configure_full_scale(max_acceleration p_gravity_code);

private:
  accelerometer::read_t driver_read();

  /**
   * @brief Changes what spi wire mode that the device will use to transfer data
   * 3 wire mode is not yet supported which is why this is locked to 4 wire mode
   *
   * @param p_spi_mode - The spi mode that the device will use
   */
  void configure_spi_mode(spi_mode p_spi_mode);

  /**
   * @brief The spi peripheral used for communication with the device.
   */
  hal::spi* m_spi;
  /**
   * @brief The output pin used to select the lis3dhtr on the spi bus
   */
  hal::output_pin* m_cs;

  /**
   * @brief The minimum and maxium g's that the device will read
   */
  hal::byte m_gscale;
};
}  // namespace hal::stm_imu

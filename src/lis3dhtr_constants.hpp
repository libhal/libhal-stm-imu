#include <libhal/units.hpp>

namespace hal::stm_imu {

/// Device identification register
constexpr hal::byte who_am_i_register = 0x0F;
/// Used to set data rate selection, power mode, and z, y, and x axis toggling
constexpr hal::byte ctrl_reg1 = 0x20;
/// Used to reboot memory and toggle fifo
constexpr hal::byte ctrl_reg4 = 0x23;
/// Used to toggle fifo
constexpr hal::byte ctrl_reg5 = 0x24;
/// Used to change fifo modes
constexpr hal::byte fifo_ctrl_reg = 0x2E;
// This is the bit mask to indicate a read on spi
constexpr hal::bit_mask spi_read_bit_mask = hal::bit_mask::from<7>();
// // this is the bit mask to indicate a auto increment address on reads and
// writes
constexpr hal::bit_mask spi_addr_inc_bit_mask = hal::bit_mask::from<6>();

/// reads from all
constexpr hal::byte read_xyz_axis = 0xA8;
/// low bits of x accelerations data
constexpr hal::byte out_x_l = 0x28;
/// high bits of x accelerations data
constexpr hal::byte out_x_h = 0x29;
/// low bits of y accelerations data
constexpr hal::byte out_y_l = 0x2A;
/// high bits of y accelerations data
constexpr hal::byte out_y_h = 0x2B;

/// low bits of z accelerations data
constexpr hal::byte out_z_l = 0x2C;
/// high bits of z accelerations data
constexpr hal::byte out_z_h = 0x2D;

}  // namespace hal::stm_imu

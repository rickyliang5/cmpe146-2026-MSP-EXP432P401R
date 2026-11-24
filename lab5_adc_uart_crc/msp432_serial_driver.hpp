#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "../hal/serial.hpp"

class msp432_serial_driver : public lab6::serial
{
public:
  static constexpr std::size_t buffer_size = 256;

  // p_smclk_hz is the actual SMCLK frequency, not necessarily SystemCoreClock.
  // Configure the clock tree before constructing this singleton A2 driver.
  msp432_serial_driver(std::uint32_t p_smclk_hz,
                       settings const& p_settings);
  ~msp432_serial_driver();
  msp432_serial_driver(msp432_serial_driver const&) = delete;
  msp432_serial_driver& operator=(msp432_serial_driver const&) = delete;

  [[nodiscard]] bool ready() const;
  static void handle_interrupt();

private:
  bool driver_configure(settings const& p_settings) override;
  void driver_write(std::span<std::uint8_t const> p_data) override;
  std::span<std::uint8_t const> driver_receive_buffer() override;
  std::size_t driver_receive_cursor() override;
  void store_received(std::uint8_t p_byte);

  inline static msp432_serial_driver* m_owner = nullptr;
  std::uint32_t m_smclk_hz;
  std::array<std::uint8_t, buffer_size> m_buffer{};
  volatile std::size_t m_cursor = 0;
};

extern "C" void EUSCIA2_IRQHandler();

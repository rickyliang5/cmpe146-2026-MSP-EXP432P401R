#pragma once

#include <cstdint>

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"

class gpio_output final : public lab1::output_pin
{
public:
  gpio_output(std::uint8_t p_port, std::uint8_t p_pin);

private:
  bool driver_configure(settings const& p_settings) override;
  void driver_level(bool p_high) override;
  bool driver_level() override;

  std::uint8_t m_port;
  std::uint8_t m_pin;
};

class spi_b0 final : public lab3::spi
{
public:
  spi_b0();

private:
  bool driver_configure(settings const& p_settings) override;
  void driver_transfer(std::span<std::uint8_t const> p_data_out,
                       std::span<std::uint8_t> p_data_in,
                       std::uint8_t p_filler) override;

  std::uint8_t transfer_byte(std::uint8_t p_byte);

  std::uint32_t m_clock_rate = 0;
};

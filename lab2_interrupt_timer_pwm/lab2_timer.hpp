#pragma once

#include <cstdint>

#include "../hal/timer.hpp"

class timer_a_clock final : public lab2::steady_clock
{
public:

  timer_a_clock();

private:
  std::uint32_t driver_frequency() override;
  std::uint64_t driver_uptime() override;

  std::uint32_t m_frequency = 0;

  std::uint64_t m_rollovers = 0;
};

void configure_rgb_pwm_pins();

class timer_a_pwm final : public lab2::pwm
{
public:

  timer_a_pwm(std::uint8_t p_channel, std::uint32_t p_frequency_hz);

private:
  std::uint32_t driver_frequency() override;
  void driver_duty_cycle(std::uint16_t p_duty_cycle) override;

  std::uint32_t m_frequency = 0;
  std::uint8_t m_channel = 0;
};

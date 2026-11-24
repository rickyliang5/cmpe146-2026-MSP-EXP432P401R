#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <cstdint>
#include <cstdio>

#include "../hal/gpio.hpp"

extern "C" void initialise_monitor_handles(void);

namespace {

struct port_pair_registers
{
  volatile std::uint8_t in[2];
  volatile std::uint8_t out[2];
  volatile std::uint8_t dir[2];
  volatile std::uint8_t ren[2];
  volatile std::uint8_t ds[2];
  volatile std::uint8_t sel0[2];
  volatile std::uint8_t sel1[2];
};

constexpr std::uintptr_t port_module_base = 0x4000'4C00;
constexpr std::uintptr_t port_pair_spacing = 0x20;
constexpr std::uint8_t port_count = 10;
constexpr std::uint8_t pins_per_port = 8;

constexpr bool is_valid_pin(std::uint8_t p_port, std::uint8_t p_pin)
{
  return p_port >= 1 && p_port <= port_count && p_pin < pins_per_port;
}

port_pair_registers* port_pair(std::uint8_t p_port)
{
  auto const pair_index = static_cast<std::uintptr_t>((p_port - 1) / 2);
  return reinterpret_cast<port_pair_registers*>(
    port_module_base + pair_index * port_pair_spacing);
}

class msp432_output_pin final : public lab1::output_pin
{
public:
  msp432_output_pin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_valid(is_valid_pin(p_port, p_pin))
    , m_registers(port_pair(m_valid ? p_port : 1))
    , m_byte(static_cast<std::uint8_t>((p_port - 1) % 2))
    , m_mask(static_cast<std::uint8_t>(1 << (p_pin % pins_per_port)))
  {
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    if (!m_valid || p_settings.open_drain ||
        p_settings.resistor != lab1::pin_resistor::none) {
      return false;
    }

    m_registers->sel0[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    m_registers->sel1[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    m_registers->ren[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    m_registers->out[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    m_registers->dir[m_byte] |= m_mask;
    return true;
  }

  void driver_level(bool p_high) override
  {
    if (p_high) {
      m_registers->out[m_byte] |= m_mask;
    } else {
      m_registers->out[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    }
  }

  bool driver_level() override
  {
    return (m_registers->in[m_byte] & m_mask) != 0;
  }

  bool m_valid;
  port_pair_registers* m_registers;
  std::uint8_t m_byte;
  std::uint8_t m_mask;
};

class msp432_input_pin final : public lab1::input_pin
{
public:
  msp432_input_pin(std::uint8_t p_port, std::uint8_t p_pin)
    : m_valid(is_valid_pin(p_port, p_pin))
    , m_registers(port_pair(m_valid ? p_port : 1))
    , m_byte(static_cast<std::uint8_t>((p_port - 1) % 2))
    , m_mask(static_cast<std::uint8_t>(1 << (p_pin % pins_per_port)))
  {
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    if (!m_valid) {
      return false;
    }

    m_registers->sel0[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    m_registers->sel1[m_byte] &= static_cast<std::uint8_t>(~m_mask);
    m_registers->dir[m_byte] &= static_cast<std::uint8_t>(~m_mask);

    switch (p_settings.resistor) {
      case lab1::pin_resistor::none:
        m_registers->ren[m_byte] &= static_cast<std::uint8_t>(~m_mask);
        return true;
      case lab1::pin_resistor::pull_up:
        m_registers->out[m_byte] |= m_mask;
        m_registers->ren[m_byte] |= m_mask;
        return true;
      case lab1::pin_resistor::pull_down:
        m_registers->out[m_byte] &= static_cast<std::uint8_t>(~m_mask);
        m_registers->ren[m_byte] |= m_mask;
        return true;
    }
    return false;
  }

  bool driver_level() override
  {
    return (m_registers->in[m_byte] & m_mask) != 0;
  }

  bool m_valid;
  port_pair_registers* m_registers;
  std::uint8_t m_byte;
  std::uint8_t m_mask;
};

struct rgb_color
{
  bool red;
  bool green;
  bool blue;
};

constexpr rgb_color off_color{ false, false, false };
constexpr rgb_color s1_color{ true, false, false };
constexpr rgb_color s2_color{ false, true, false };
constexpr rgb_color both_color{ false, false, true };

}

int main()
{
  MAP_WDT_A_holdTimer();

  msp432_output_pin red{ 2, 0 };
  msp432_output_pin green{ 2, 1 };
  msp432_output_pin blue{ 2, 2 };
  msp432_input_pin s1{ 1, 1 };
  msp432_input_pin s2{ 1, 4 };

  red.configure({});
  green.configure({});
  blue.configure({});
  s1.configure({ .resistor = lab1::pin_resistor::pull_up });
  s2.configure({ .resistor = lab1::pin_resistor::pull_up });

  initialise_monitor_handles();
  std::printf("Hello, World\n");

  while (true) {
    bool const s1_pressed = !s1.level();
    bool const s2_pressed = !s2.level();

    rgb_color color = off_color;
    if (s1_pressed && s2_pressed) {
      color = both_color;
    } else if (s1_pressed) {
      color = s1_color;
    } else if (s2_pressed) {
      color = s2_color;
    }

    red.level(color.red);
    green.level(color.green);
    blue.level(color.blue);
  }
}

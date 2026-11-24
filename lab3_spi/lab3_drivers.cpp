#include "lab3_drivers.hpp"

#include <cstddef>
#include <cstdint>

namespace {

constexpr std::uintptr_t port_base = 0x4000'4C00;
constexpr std::uintptr_t block_stride = 0x20;
constexpr std::uintptr_t eusci_b0_base = 0x4000'2000;

constexpr std::uint32_t smclk_frequency = 3'000'000;

struct port_pair_registers
{
  std::uint8_t volatile in[2];
  std::uint8_t volatile out[2];
  std::uint8_t volatile dir[2];
  std::uint8_t volatile ren[2];
  std::uint8_t volatile ds[2];
  std::uint8_t volatile sel0[2];
  std::uint8_t volatile sel1[2];
};

static_assert(offsetof(port_pair_registers, out) == 0x02);
static_assert(offsetof(port_pair_registers, dir) == 0x04);
static_assert(offsetof(port_pair_registers, sel0) == 0x0A);
static_assert(offsetof(port_pair_registers, sel1) == 0x0C);

struct eusci_b_registers
{
  std::uint16_t volatile ctlw0;
  std::uint16_t volatile ctlw1;
  std::uint16_t volatile reserved0;
  std::uint16_t volatile brw;
  std::uint16_t volatile statw;
  std::uint16_t volatile tbcnt;
  std::uint16_t volatile rxbuf;
  std::uint16_t volatile txbuf;
  std::uint16_t volatile reserved1[2];
  std::uint16_t volatile i2coa0;
  std::uint16_t volatile i2coa1;
  std::uint16_t volatile i2coa2;
  std::uint16_t volatile i2coa3;
  std::uint16_t volatile addrx;
  std::uint16_t volatile addmask;
  std::uint16_t volatile i2csa;
  std::uint16_t volatile reserved2[4];
  std::uint16_t volatile ie;
  std::uint16_t volatile ifg;
  std::uint16_t volatile iv;
};

static_assert(offsetof(eusci_b_registers, brw) == 0x06);
static_assert(offsetof(eusci_b_registers, rxbuf) == 0x0C);
static_assert(offsetof(eusci_b_registers, txbuf) == 0x0E);
static_assert(offsetof(eusci_b_registers, i2csa) == 0x20);
static_assert(offsetof(eusci_b_registers, ifg) == 0x2C);
static_assert(sizeof(eusci_b_registers) == 0x30);

constexpr std::uint16_t uc_ckph = 1 << 15;
constexpr std::uint16_t uc_ckpl = 1 << 14;
constexpr std::uint16_t uc_msb = 1 << 13;
constexpr std::uint16_t uc_mst = 1 << 11;
constexpr std::uint16_t uc_sync = 1 << 8;
constexpr std::uint16_t uc_ssel_smclk = 0b10 << 6;
constexpr std::uint16_t uc_swrst = 1 << 0;

constexpr std::uint16_t uc_rxifg = 1 << 0;
constexpr std::uint16_t uc_txifg = 1 << 1;

constexpr std::uint8_t p1_spi_mask = (1 << 5) | (1 << 6) | (1 << 7);

auto* const eusci_b0 = reinterpret_cast<eusci_b_registers*>(eusci_b0_base);

struct pin_access
{
  port_pair_registers* registers;
  std::size_t port_index;
  std::uint8_t pin_mask;
};

pin_access resolve(std::uint8_t p_port, std::uint8_t p_pin)
{
  auto const pair = static_cast<std::uintptr_t>(p_port - 1) / 2;
  return {
    reinterpret_cast<port_pair_registers*>(port_base + (pair * block_stride)),
    static_cast<std::size_t>((p_port - 1) % 2),
    static_cast<std::uint8_t>(1 << p_pin),
  };
}

}  // namespace

gpio_output::gpio_output(std::uint8_t p_port, std::uint8_t p_pin)
  : m_port(p_port)
  , m_pin(p_pin)
{
}

bool gpio_output::driver_configure(settings const& p_settings)
{
  if (p_settings.open_drain) {
    return false;
  }
  if (p_settings.resistor != lab1::pin_resistor::none) {
    return false;
  }

  auto const pin = resolve(m_port, m_pin);

  pin.registers->sel0[pin.port_index] &=
    static_cast<std::uint8_t>(~pin.pin_mask);
  pin.registers->sel1[pin.port_index] &=
    static_cast<std::uint8_t>(~pin.pin_mask);
  pin.registers->dir[pin.port_index] |= pin.pin_mask;

  return true;
}

void gpio_output::driver_level(bool p_high)
{
  auto const pin = resolve(m_port, m_pin);

  if (p_high) {
    pin.registers->out[pin.port_index] |= pin.pin_mask;
  } else {
    pin.registers->out[pin.port_index] &=
      static_cast<std::uint8_t>(~pin.pin_mask);
  }
}

bool gpio_output::driver_level()
{
  auto const pin = resolve(m_port, m_pin);
  return (pin.registers->in[pin.port_index] & pin.pin_mask) != 0;
}

spi_b0::spi_b0()
{
  eusci_b0->ctlw0 = uc_swrst;

  auto const pin = resolve(1, 0);
  pin.registers->sel0[pin.port_index] |= p1_spi_mask;
  pin.registers->sel1[pin.port_index] &=
    static_cast<std::uint8_t>(~p1_spi_mask);

  configure(settings{});
}

bool spi_b0::driver_configure(settings const& p_settings)
{
  if (p_settings.clock_rate == 0) {
    return false;
  }

  std::uint16_t phase_polarity = 0;
  switch (p_settings.bus_mode) {
    case mode::m0:
      phase_polarity = uc_ckph;
      break;
    case mode::m1:
      phase_polarity = 0;
      break;
    case mode::m2:
      phase_polarity = uc_ckph | uc_ckpl;
      break;
    case mode::m3:
      phase_polarity = uc_ckpl;
      break;
  }

  auto divider =
    (smclk_frequency + p_settings.clock_rate - 1) / p_settings.clock_rate;
  if (divider == 0) {
    divider = 1;
  }
  if (divider > 0xFFFF) {
    divider = 0xFFFF;
  }

  eusci_b0->ctlw0 = uc_swrst;
  eusci_b0->ctlw0 = static_cast<std::uint16_t>(
    phase_polarity | uc_msb | uc_mst | uc_sync | uc_ssel_smclk | uc_swrst);
  eusci_b0->brw = static_cast<std::uint16_t>(divider);
  eusci_b0->ctlw0 &= static_cast<std::uint16_t>(~uc_swrst);

  m_clock_rate = smclk_frequency / divider;

  return true;
}

std::uint8_t spi_b0::transfer_byte(std::uint8_t p_byte)
{
  while ((eusci_b0->ifg & uc_txifg) == 0) {
    continue;
  }
  eusci_b0->txbuf = p_byte;

  while ((eusci_b0->ifg & uc_rxifg) == 0) {
    continue;
  }
  return static_cast<std::uint8_t>(eusci_b0->rxbuf);
}

void spi_b0::driver_transfer(std::span<std::uint8_t const> p_data_out,
                             std::span<std::uint8_t> p_data_in,
                             std::uint8_t p_filler)
{
  auto const count =
    (p_data_out.size() > p_data_in.size()) ? p_data_out.size()
                                           : p_data_in.size();

  for (std::size_t i = 0; i < count; i++) {
    auto const out = (i < p_data_out.size()) ? p_data_out[i] : p_filler;
    auto const in = transfer_byte(out);

    if (i < p_data_in.size()) {
      p_data_in[i] = in;
    }
  }
}

namespace std {
void __glibcxx_assert_fail(char const*, int, char const*, char const*)
{
  while (true) {
    continue;
  }
}
}  // namespace std

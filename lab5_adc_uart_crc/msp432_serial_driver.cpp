// cspell:ignore EUSCIA RXIFG TXIFG RXBUF TXBUF CTLW MCTL BRW PRIMASK
#include "msp432_serial_driver.hpp"

#include <ti/devices/msp432p4xx/inc/msp.h>

msp432_serial_driver::msp432_serial_driver(std::uint32_t p_smclk_hz,
                                           settings const& p_settings)
  : m_smclk_hz(p_smclk_hz)
{
  driver_configure(p_settings);
}

msp432_serial_driver::~msp432_serial_driver()
{
  auto const mask = __get_PRIMASK();
  __disable_irq();
  if (m_owner == this) {
    EUSCI_A2->IE = 0;
    NVIC_DisableIRQ(EUSCIA2_IRQn);
    NVIC_ClearPendingIRQ(EUSCIA2_IRQn);
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST;
    m_owner = nullptr;
  }
  __set_PRIMASK(mask);
}

bool msp432_serial_driver::ready() const
{
  return m_owner == this;
}

bool msp432_serial_driver::driver_configure(settings const& p_settings)
{
  if (p_settings.baud_rate != 9'600 ||
      p_settings.parity != decltype(p_settings.parity)::none ||
      p_settings.stop != settings::stop_bits::one) {
    return false;
  }

  // With oversampling, BRW supplies the groups of 16 clocks and BRF
  // supplies the remaining clocks. For the lab's 3 MHz / 9600 baud,
  // rounding to a whole clock keeps the baud error below 0.2%.
  auto const baud_rate = p_settings.baud_rate;
  auto const clocks_per_bit =
    (std::uint64_t{m_smclk_hz} + baud_rate / 2) / baud_rate;
  auto const prescaler = clocks_per_bit / 16;
  auto const realized_clock = clocks_per_bit * baud_rate;
  auto const clock_error = realized_clock > m_smclk_hz
                             ? realized_clock - m_smclk_hz
                             : m_smclk_hz - realized_clock;
  if (prescaler == 0 || prescaler > UINT16_MAX ||
      clock_error * 100 > realized_clock) {
    return false;
  }
  auto const modulation = ((clocks_per_bit % 16) << 4) | EUSCI_A_MCTLW_OS16;

  auto const mask = __get_PRIMASK();
  __disable_irq();
  if (m_owner != nullptr && m_owner != this) {
    __set_PRIMASK(mask);
    return false;
  }
  EUSCI_A2->IE = 0;
  NVIC_DisableIRQ(EUSCIA2_IRQn);
  EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;
  // P3.2 = RX (J1.3), P3.3 = TX (J1.4), primary module function.
  P3->SEL0 |= 0x0C;
  P3->SEL1 &= ~0x0C;
  P3->DIR = (P3->DIR & ~0x04) | 0x08;
  EUSCI_A2->BRW = static_cast<std::uint16_t>(prescaler);
  EUSCI_A2->MCTLW = static_cast<std::uint16_t>(modulation);
  m_cursor = 0;
  m_owner = this;
  NVIC_ClearPendingIRQ(EUSCIA2_IRQn);
  // SYNC/MODE/MSB/SEVENBIT remain clear: asynchronous, LSB first, 8 bits.
  EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
  EUSCI_A2->IE = EUSCI_A_IE_RXIE;
  NVIC_EnableIRQ(EUSCIA2_IRQn);
  __set_PRIMASK(mask);
  return true;
}

void msp432_serial_driver::driver_write(std::span<std::uint8_t const> p_data)
{
  if (!ready()) {
    return;
  }
  for (auto const byte : p_data) {
    while ((EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG) == 0) {
      // Polled transmit: no DriverLib, TX interrupt or FreeRTOS calls.
    }
    EUSCI_A2->TXBUF = byte;
  }
}

void msp432_serial_driver::store_received(std::uint8_t p_byte)
{
  auto const cursor = m_cursor;
  m_buffer[cursor] = p_byte;
  // Publish data before its next-write cursor on this single-core Cortex-M4.
  __DMB();
  m_cursor = (cursor + 1) % buffer_size;
}

void msp432_serial_driver::handle_interrupt()
{
  if (m_owner == nullptr) {
    EUSCI_A2->IE &= ~EUSCI_A_IE_RXIE;
    return;
  }
  // Reading RXBUF acknowledges RXIFG. Drain any byte arriving during the ISR.
  while ((EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) != 0) {
    auto const byte = static_cast<std::uint8_t>(EUSCI_A2->RXBUF);
    m_owner->store_received(byte);
  }
}

std::span<std::uint8_t const> msp432_serial_driver::driver_receive_buffer()
{
  return { m_buffer.data(), m_buffer.size() };
}

std::size_t msp432_serial_driver::driver_receive_cursor()
{
  auto const cursor = m_cursor;
  __DMB();
  return cursor;
}

extern "C" void EUSCIA2_IRQHandler()
{
  msp432_serial_driver::handle_interrupt();
}

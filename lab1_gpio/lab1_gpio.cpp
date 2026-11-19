#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <cstdint>
#include <cstdio>

#include "../hal/gpio.hpp"

// Routes std::printf output to the CCS Console over JTAG semihosting.
// Requires the debugger to stay connected and running - output is lost if
// you disconnect or power-cycle the board instead of debugging it live.
extern "C" void initialise_monitor_handles(void);


class msp432_input : public lab1::input_pin
{
  msp432_input(int port, int pin)
  {
    if (port & 1) {
      mask = 1 << pin;
    } else {
      mask = 1 << (pin + 8);
    }
  }

  uint16_t mask;

  bool driver_configure(settings const& p_settings) override
  {
    return false;
  }
  bool driver_level() override
  {
    return false;
  }
};

int main()
{
  // Stop the watchdog timer. Without this, the device resets a few seconds
  // after boot. This is the only DriverLib call allowed in this lab - it is
  // boilerplate, not part of the GPIO driver you are writing.
  MAP_WDT_A_holdTimer();

  // Red LED on P2.0. Base address 0x4000_4C00 is the Port Module, from
  // datasheet Table 6-1 (section 6.3.3); the register offsets are TRM
  // Table 10-3. P1 and P2 share the 16-bit PA registers - P1 is the low
  // byte and P2 the high byte - so P2.0 is bit 8.
  constexpr std::uintptr_t port_base = 0x4000'4C00;  // PA = P1/P2
  constexpr std::uint16_t red_led = 1 << 8;          // P2.0

  auto* pasel0 = reinterpret_cast<volatile std::uint16_t*>(port_base + 0x0A);
  auto* pasel1 = reinterpret_cast<volatile std::uint16_t*>(port_base + 0x0C);
  auto* padir = reinterpret_cast<volatile std::uint16_t*>(port_base + 0x04);
  auto* paout = reinterpret_cast<volatile std::uint16_t*>(port_base + 0x02);

  *pasel0 &= ~red_led;  // SEL0 = 0 and SEL1 = 0 selects GPIO
  *pasel1 &= ~red_led;
  *padir |= red_led;    // 1 = output
  *paout |= red_led;    // 1 = high, so the LED lights

  // Semihosting goes after the LED. initialise_monitor_handles() and printf
  // both issue BKPT instructions that only a connected debugger services -
  // run the board standalone and they hang, so nothing after them executes.
  // The LED is driven first so it lights either way.
  initialise_monitor_handles();

  std::printf("Hello, World\n");

  // TODO(lab1, step 1): Implement the `lab1::output_pin` interface
  // (../hal/gpio.hpp) for the MSP432P401R, register-level - no DriverLib.
  // Use it to drive the on-board RGB LED. See README.md for which port/pin
  // each LED color is wired to.

  // TODO(lab1, step 2): Implement the `lab1::input_pin` interface
  // (../hal/gpio.hpp) for the MSP432P401R, register-level - no DriverLib.
  // Use it to read the two push-buttons (S1, S2). See README.md for which
  // port/pin each button is wired to.
  while (true) {
    

  }
}

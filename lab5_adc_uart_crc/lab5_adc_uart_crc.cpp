// cspell:ignore INIRES RESR DIRB MSB GENRDY CTLBASE XFERMODE semphr EUSCIA
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include "../hal/adc.hpp"
#include "../hal/crc.hpp"
#include "../hal/gpio.hpp"
#include "../hal/timer.hpp"
#include "msp432_serial_driver.hpp"

// GNU rdimon provides printf output in the CCS CIO Console. The debugger
// check prevents semihosting calls from stopping a board running on its own.
extern "C" void initialise_monitor_handles(void);

// GCC 15 enables _GLIBCXX_ASSERTIONS in unoptimized builds, so std::array
// bounds checks call this function. It normally lives in libstdc++, which
// this project doesn't link, so the link fails without it. A failed bounds
// check halts here like a configASSERT: pause the debugger and check the
// call stack.
namespace std {
[[noreturn]] void __glibcxx_assert_fail(char const*,
                                        int,
                                        char const*,
                                        char const*) noexcept
{
  __disable_irq();
  while (true) {
    continue;
  }
}
}  // namespace std

// The compiler calls these around the first-time construction of a `static`
// object inside a function (the drivers in main()). They also normally live
// in libstdc++. All such objects here are constructed in main() before the
// scheduler starts, so there is only one thread and no locking is needed:
// bit 0 of the guard word just records "already constructed".
extern "C" int __cxa_guard_acquire(std::uint32_t* p_guard)
{
  return (*p_guard & 1U) == 0;
}

extern "C" void __cxa_guard_release(std::uint32_t* p_guard)
{
  *p_guard = 1;
}

extern "C" void __cxa_guard_abort(std::uint32_t*)
{
}

// The kernel library's tickless-idle hook (TI's PowerMSP432_freertos.c) calls
// this function, which normally comes from the TI Drivers library that this
// project does not link. Depending on the order the linker reads the kernel
// library, that hook can be chosen over FreeRTOS's built-in one, and the link
// then fails without this. Weak, so a real implementation would replace it.
// It sleeps the CPU until the next interrupt, like the built-in hook does.
extern "C" __attribute__((weak)) void Power_idleFunc(void)
{
  __WFI();
}

namespace {
// =============================================================================
// Debug output
// =============================================================================

bool semihosting_ready = false;
std::array<char, 256> semihosting_stdout_buffer{};

bool debugger_connected()
{
  return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0;
}

// Semihosting printf halts the core when no debugger is attached, so all
// output goes through debug_printf, which only prints when it is safe. The
// scheduler is suspended while printing so no other task can switch in
// partway through a semihosting call. Interrupts stay enabled, so the UART
// keeps receiving in the background.
template<typename... Args>
void debug_printf(char const* p_format, Args... p_args)
{
  if (semihosting_ready && debugger_connected()) {
    vTaskSuspendAll();
    std::printf(p_format, p_args...);
    xTaskResumeAll();
  }
}

// Progress counters for checking the pipeline without the console: add these
// to the CCS Expressions view and watch them while the board runs.
volatile std::uint32_t debug_requests_received = 0;
volatile std::uint32_t debug_requests_merged = 0;
volatile std::uint32_t debug_batches_sent = 0;

enum class device_role : std::uint8_t
{
  sensor,
  display,
};

// Change this before building each board.
constexpr device_role this_device = device_role::display;

// Single-board test of the sensor role. Set to true, jumper J1.4 (TX) to
// J1.3 (RX) on the sensor board, and rebuild: a test task then sends the
// request byte every 2 s, so the UART, ADC, conversion and CRC all run with
// no display board. The packet itself is not transmitted in this mode,
// because any 0xA5 byte inside it would loop back as a fake request.
// Must be false for normal two-board operation.
constexpr bool loopback_test = false;

// SystemInit leaves MCLK and SMCLK at 3 MHz. The UART baud rate, the ADC
// sample time and the benchmark clock all assume this.
constexpr std::uint32_t system_clock_hz = 3'000'000;

// =============================================================================
// Packet format (both boards must agree on this)
//
//   64 x temperature_sample_t: int16_t in units of 0.1 degC (235 = 23.5 degC),
//   native (little-endian) byte order
//   4-byte CRC-32 of those sample bytes, least significant byte first
//
//   64 * 2 + 4 = 132 bytes
// =============================================================================

// The one-byte message the display board sends to ask the sensor board for
// a fresh batch of readings.
constexpr std::uint8_t sample_request_command = 0xA5; // value sent over UART to be passed through eUSCI_A2 UART's data registers

constexpr std::size_t samples_per_batch = 64;

using temperature_sample_t = std::int16_t;
using temperature_batch = std::array<temperature_sample_t, samples_per_batch>;

constexpr std::size_t sample_bytes = sizeof(temperature_batch);
constexpr std::size_t crc_size = sizeof(std::uint32_t);
constexpr std::size_t packet_size = sample_bytes + crc_size;

/// How long the display board waits for a reply before giving up. The
/// sensor's semihosting printfs (debugger attached) can delay its reply by
/// several hundred ms, so this leaves plenty of margin over the ~140 ms
/// the 132-byte packet itself takes at 9600 baud.
constexpr TickType_t reply_timeout = pdMS_TO_TICKS(2000);

/// LED color bands, in 0.1 degC: below 20.0 is cold, below 28.0 is warm.
constexpr std::int32_t warm_tenths = 200;
constexpr std::int32_t hot_tenths = 280;

// =============================================================================
// Role 1: ADC driver (DriverLib)
// =============================================================================

// The task currently asleep inside msp432_adc_driver::read(). read() stores
// it before starting the conversion; DMA_INT1_IRQHandler() reads it to know
// which task to wake.
TaskHandle_t volatile adc_waiting_task = nullptr;

// The DMA controller keeps each channel's settings in this table in RAM. The
// hardware requires it to sit on a 1024-byte boundary.
alignas(1024) DMA_ControlTable dma_control_table[32];

// The only DMA channel the ADC14 can trigger.
constexpr std::uint32_t adc_dma_channel = 7;

// The DMA controller moves at most this many items per transfer.
constexpr std::size_t dma_max_transfer_items = 1024;

/**
 * @brief ADC14 driver for the external TMP36 on P6.1 (input A14).
 *
 * The ADC14 converts A14 over and over into conversion memory 0, and DMA
 * channel 7 copies each result into the caller's buffer. The calling task
 * sleeps for the whole transfer and is woken by the DMA's completion
 * interrupt, so the CPU never polls the ADC.
 */
class msp432_adc_driver final : public lab5::adc16_stream
{
public:
  /// Reference voltage the samples are measured against, in millivolts.
  static constexpr std::int32_t reference_millivolts = 2500;
  /// Native resolution of the ADC14 before upscaling to 16 bits.
  static constexpr unsigned resolution_bits = 14;

  msp432_adc_driver()
  {
    // P6.1 is the TMP36's VOUT. Its analog function is A14.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                                               GPIO_PIN1,
                                               GPIO_TERTIARY_MODULE_FUNCTION);

    // Internal 2.5 V reference. Wait for it to settle (bounded, so a fault
    // here cannot hang the board at startup).
    REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    REF_A_enableReferenceVoltage();
    for (std::uint32_t tries = 0;
         tries < 100'000 && (REF_A->CTL0 & REF_A_CTL0_GENRDY) == 0;
         tries++) {
    }

    // ADC clocked from MCLK (3 MHz), unsigned 14-bit results.
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK,
                     ADC_PREDIVIDER_1,
                     ADC_DIVIDER_1,
                     ADC_NOROUTE);
    ADC14_setResolution(ADC_14BIT);
    ADC14_setResultFormat(ADC_UNSIGNED_BINARY);

    // Conversion memory 0 measures A14 against the internal reference, and is
    // converted repeatedly once triggered.
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0,
                                    ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                    ADC_INPUT_A14,
                                    false);

    // 64 ADC clocks = 21 us of sample time at 3 MHz, comfortably longer than
    // the ADC14 needs for a low-impedance source like the TMP36.
    ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_64, ADC_PULSE_WIDTH_64);

    // Start each conversion automatically when the previous one finishes.
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // DMA channel 7: triggered by the ADC, 16-bit items, always read from the
    // same register, write to consecutive buffer slots, one item per trigger.
    DMA_enableModule();
    DMA_setControlBase(dma_control_table);
    DMA_assignChannel(DMA_CH7_ADC14);
    DMA_disableChannelAttribute(DMA_CH7_ADC14,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                  UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                            UDMA_ARB_1);

    // DMA_INT1 fires when channel 7 finishes a transfer. Its handler calls
    // into FreeRTOS, so it must run at or below the kernel's maximum
    // interrupt priority (configMAX_SYSCALL_INTERRUPT_PRIORITY, level 1).
    DMA_assignInterrupt(DMA_INT1, adc_dma_channel);
    Interrupt_setPriority(INT_DMA_INT1, 2 << 5);
    Interrupt_enableInterrupt(INT_DMA_INT1);
  }

private:
  void driver_read(std::span<std::uint16_t> p_buffer) override
  {
    // One DMA transfer per chunk. A buffer of up to 1024 samples (the lab
    // uses 64) is a single transfer and a single task sleep.
    while (!p_buffer.empty()) {
      auto const chunk =
        p_buffer.first(std::min(p_buffer.size(), dma_max_transfer_items));
      transfer(chunk);
      p_buffer = p_buffer.subspan(chunk.size());
    }
  }

  /// Fill p_chunk with one DMA transfer, sleeping until it completes.
  void transfer(std::span<std::uint16_t> p_chunk)
  {
    // 1. Make sure nothing is left running from a previous read.
    stop();

    // 2. Point the DMA at this buffer. The channel switches itself off after
    //    every completed transfer, so it is set up again each time.
    DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                           UDMA_MODE_BASIC,
                           const_cast<void*>(
                             static_cast<void const volatile*>(&ADC14->MEM[0])),
                           p_chunk.data(),
                           p_chunk.size());

    // 3. Save the task to wake. This must happen before the transfer starts,
    //    because the interrupt can fire any time after that.
    adc_waiting_task = xTaskGetCurrentTaskHandle();

    // 4. Enable the DMA channel and start converting.
    DMA_enableChannel(adc_dma_channel);
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();

    // 5. Sleep until DMA_INT1_IRQHandler() reports the buffer is full.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    adc_waiting_task = nullptr;

    // 6. Stop converting, then upscale the 14-bit samples to 16 bits by bit
    //    duplication, so 0 stays 0 and the maximum sample becomes 0xFFFF.
    stop();
    for (auto& sample : p_chunk) {
      sample = static_cast<std::uint16_t>(
        (sample << (16 - resolution_bits)) |
        (sample >> (2 * resolution_bits - 16)));
    }
  }

  /// Stop repeating conversions and wait for the ADC to go idle.
  void stop()
  {
    ADC14_disableConversion();
    DMA_disableChannel(adc_dma_channel);
    while (ADC14_isBusy()) {
    }
    // Discard the last result so the next transfer cannot start with a stale
    // sample.
    ADC14_clearInterruptFlag(ADC_INT0);
  }
};

/**
 * @brief Split a temperature in 0.1 degC units into printable parts.
 *
 * newlib-nano's printf has no %f, so print as "%s%ld.%ld degC" with
 * sign, whole and tenth: 268 -> "26.8", -5 -> "-0.5".
 */
struct printable_temperature
{
  char const* sign;
  long whole;
  long tenth;
};

printable_temperature to_printable(std::int32_t p_tenths)
{
  std::int32_t const magnitude = p_tenths < 0 ? -p_tenths : p_tenths;
  return { p_tenths < 0 ? "-" : "", magnitude / 10, magnitude % 10 };
}

/**
 * @brief Convert one 16-bit upscaled TMP36 sample to 0.1 degC units.
 *
 * The ADC code is first turned into millivolts at the TMP36's VOUT pin
 * (2.5 V reference, 14-bit code). The TMP36 outputs 500 mV at 0 degC and
 * rises 10 mV per degC, so one millivolt above 500 mV is 0.1 degC.
 */
temperature_sample_t to_temperature(std::uint16_t p_sample)
{
  constexpr std::int32_t full_scale = 1 << msp432_adc_driver::resolution_bits;
  constexpr std::int32_t millivolts_at_zero_celsius = 500;

  // Back to the ADC's native 14 bits.
  std::int32_t const code =
    p_sample >> (16 - msp432_adc_driver::resolution_bits);
  // Rounded to the nearest millivolt.
  std::int32_t const millivolts =
    (code * msp432_adc_driver::reference_millivolts + full_scale / 2) /
    full_scale;

  return static_cast<temperature_sample_t>(millivolts -
                                           millivolts_at_zero_celsius);
}

// =============================================================================
// Role 3: CRC drivers
// =============================================================================

/**
 * @brief Register-level driver for the MSP432 on-chip CRC32 accelerator.
 *
 * Computes the standard CRC-32 (reflected polynomial 0xEDB88320, seed
 * 0xFFFFFFFF, final XOR 0xFFFFFFFF), so its result matches crc32_software.
 * This is the same setup as TI's crc32_32-bit_signature_calculation example:
 *
 * - Input: bytes go to DI32. DIRB32 bit-reverses each byte first, which does
 *   not match the reflected CRC-32, so it is not used.
 * - Result: RESR32 is INIRES32 with all 32 bits reversed (TRM 13.3.0.5).
 *   The module computes in the standard's MSB-first order, so the reflected
 *   CRC-32 value is in RESR32, not INIRES32.
 * - Final XOR: the module does not apply one, so it is done in software.
 *
 * The device has one CRC32 module. Use one instance at a time: constructing
 * or resetting an instance restarts the calculation for all of them.
 */
class msp432_crc32_hardware final : public lab6::crc32
{
public:
  /**
   * @brief Load the CRC-32 seed so the module is ready for `update()`.
   *
   * The CRC32 module has no clock or enable bit, so loading the seed is the
   * only setup it needs.
   */
  msp432_crc32_hardware()
  {
    driver_reset();
  }

private:
  static constexpr std::uint32_t initial_value = 0xFFFF'FFFFU;

  void driver_reset() override
  {
    // Writing INIRES32 loads the seed and restarts the calculation. It resets
    // to 0x0000, so the seed must always be written. 0xFFFFFFFF is the same
    // when bit-reversed, so it needs no conversion.
    CRC32->INIRES32_HI = static_cast<std::uint16_t>(initial_value >> 16);
    CRC32->INIRES32_LO = static_cast<std::uint16_t>(initial_value);
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    // An 8-bit write to DI32 feeds one byte. DI32 is a 16-bit register, so
    // take its address as a byte pointer (as DriverLib's CRC32 code does).
    auto* const data_in =
      reinterpret_cast<volatile std::uint8_t*>(&CRC32->DI32);
    for (auto const byte : p_data) {
      *data_in = byte;
    }
  }

  std::uint32_t driver_checksum() override
  {
    std::uint32_t const result =
      static_cast<std::uint32_t>(CRC32->RESR32_HI) << 16 | CRC32->RESR32_LO;
    return result ^ initial_value;
  }
};

// Supplied software CRC-32 reference. Do not modify it. Use it for packet
// CRCs and compare its benchmark result with msp432_crc32_hardware.
class crc32_software : public lab6::crc32
{
private:
  void driver_reset() override
  {
    m_crc = initial_value;
  }

  void driver_update(std::span<std::uint8_t const> data) override
  {
    for (auto const byte : data) {
      m_crc ^= byte;
      for (int bit = 0; bit < 8; ++bit) {
        std::uint32_t const mask = -(m_crc & 1U);
        m_crc = (m_crc >> 1) ^ (polynomial & mask);
      }
    }
  }

  std::uint32_t driver_checksum() override
  {
    return m_crc ^ initial_value;
  }

  static constexpr std::uint32_t polynomial = 0xEDB8'8320U;
  static constexpr std::uint32_t initial_value = 0xFFFF'FFFFU;
  std::uint32_t m_crc = initial_value;
};

// =============================================================================
// GPIO pins and the benchmark clock
// =============================================================================

/// lab1::output_pin on any MSP432 pin, set up in the constructor.
class msp432_output_pin final : public lab1::output_pin
{
public:
  msp432_output_pin(std::uint_fast8_t p_port, std::uint_fast16_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    MAP_GPIO_setOutputLowOnPin(m_port, m_pin);  // start off
    MAP_GPIO_setAsOutputPin(m_port, m_pin);
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    return !p_settings.open_drain;  // the MSP432 has no open drain mode
  }

  void driver_level(bool p_high) override
  {
    if (p_high) {
      MAP_GPIO_setOutputHighOnPin(m_port, m_pin);
    } else {
      MAP_GPIO_setOutputLowOnPin(m_port, m_pin);
    }
  }

  bool driver_level() override
  {
    return MAP_GPIO_getInputPinValue(m_port, m_pin) == GPIO_INPUT_PIN_HIGH;
  }

  std::uint_fast8_t m_port;
  std::uint_fast16_t m_pin;
};

/// lab1::input_pin with a pull-up, set up in the constructor. The LaunchPad
/// buttons connect the pin to GND when pressed, so pressed reads LOW.
class msp432_input_pin final : public lab1::input_pin
{
public:
  msp432_input_pin(std::uint_fast8_t p_port, std::uint_fast16_t p_pin)
    : m_port(p_port)
    , m_pin(p_pin)
  {
    MAP_GPIO_setAsInputPinWithPullUpResistor(m_port, m_pin);
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    return p_settings.resistor == lab1::pin_resistor::pull_up;  // only mode used
  }

  bool driver_level() override
  {
    return MAP_GPIO_getInputPinValue(m_port, m_pin) == GPIO_INPUT_PIN_HIGH;
  }

  std::uint_fast8_t m_port;
  std::uint_fast16_t m_pin;
};

/**
 * @brief lab2::steady_clock on Timer32 module 1, counting MCLK cycles.
 *
 * The timer free-runs as a 32-bit down counter with no interrupt. uptime()
 * extends it to 64 bits by noticing each wrap, so it must be called at least
 * once per wrap (about 24 minutes at 3 MHz) to stay continuous. Differences
 * between two readings taken close together, which is all the CRC benchmark
 * needs, are always correct.
 */
class msp432_steady_clock final : public lab2::steady_clock
{
public:
  explicit msp432_steady_clock(std::uint32_t p_mclk_hz)
    : m_frequency(p_mclk_hz)
  {
    TIMER32_1->LOAD = 0xFFFF'FFFFU;
    // 32-bit, free-running, divide by 1, interrupt disabled.
    TIMER32_1->CONTROL = TIMER32_CONTROL_ENABLE | TIMER32_CONTROL_SIZE;
  }

private:
  std::uint32_t driver_frequency() override
  {
    return m_frequency;
  }

  std::uint64_t driver_uptime() override
  {
    // The counter counts down, so invert it to get a value that counts up.
    std::uint32_t const now = ~TIMER32_1->VALUE;
    if (now < m_last) {
      m_wraps++;
    }
    m_last = now;
    return (static_cast<std::uint64_t>(m_wraps) << 32) | now;
  }

  std::uint32_t m_frequency;
  std::uint32_t m_last = 0;
  std::uint32_t m_wraps = 0;
};

// =============================================================================
// Sensor board: semaphore and queue
// =============================================================================

// request_task -> sampler_task: binary semaphore, carries no data, only
// "take a batch now".
StaticSemaphore_t sample_request_storage; // reserves RAM for semaphore's memory
SemaphoreHandle_t sample_request = nullptr; // name the task uses for the semaphore (empty until main creates it)

// sampler_task -> transmit_task: one slot holding a full copy of 64
// converted samples.
constexpr UBaseType_t batch_queue_length = 1; // sets batch queue to 1 slot
StaticQueue_t batch_queue_control; // reserves RAM for queue's memory
alignas(temperature_batch) // makes buffer start at an even address (multiple of 3)
  std::array<std::uint8_t, batch_queue_length * sizeof(temperature_batch)> // declares fixed size array of raw bytes (64 reading * 2 = 128 bytes) 
    batch_queue_storage{}; // name of 128 byte array that holds queues contents
QueueHandle_t batch_queue = nullptr; // names the handle task for the queue, starts out as nothing 

// =============================================================================
// Sensor board: tasks
// =============================================================================

// Watches the UART receive buffer for sample_request_command and gives the
// sample_request semaphore. Only reads from the serial port.
void request_task(void* p_parameter)
{
  auto& port = *static_cast<lab6::serial*>(p_parameter); // turns the generic parameter back into UART object
  auto const buffer = port.receive_buffer(); //gets read only access to the 256 byte buffer the UART interrupt fills
  // Start from the current position: ignore anything received before startup.
  std::size_t last_cursor = port.receive_cursor(); // remebers where new data will start, ignoring anything older

  while (true) {
    std::size_t const cursor = port.receive_cursor(); // checking where in the recive buffer the UART has written
    // Walk every new byte, wrapping around the circular buffer.
    while (last_cursor != cursor) { // checks each new byte in buffer until UART position
      if (buffer[last_cursor] == sample_request_command) { // is the byte a command request?
        if (xSemaphoreGive(sample_request) == pdTRUE) { // raise the flag to wake sampler_task; false if a request is already pending
          debug_requests_received = debug_requests_received + 1; // record request was accepted
          debug_printf("request: sample request received\n"); 
        } else {
          // Count already 1: a batch is pending, so this request merges with it.
          debug_requests_merged = debug_requests_merged + 1; // flag is already raised so count the request as merged with earlier waiting request
          debug_printf("request: batch already pending, request merged\n");
        }
      }
      last_cursor = (last_cursor + 1) % buffer.size(); // move to next byt in buffer
      // ^^ wraps from slot 255 to 0
    }
    // 5 ms poll: at 9600 baud the 256-byte buffer takes ~266 ms to lap.
    vTaskDelay(pdMS_TO_TICKS(5)); // sleep for 5ms so other task can run
  }
}

// Waits for a request, takes one batch of samples in a single DMA transfer,
// converts them to temperatures, and hands a copy to transmit_task.
void sampler_task(void* p_parameter) // function that waits for sample request, then takes 1 batch and queues it 
{
  auto& adc = *static_cast<lab5::adc16_stream*>(p_parameter); // turns parameter into ADC object
  std::array<std::uint16_t, samples_per_batch> raw{}; // creates array of 64 raw 16 bit ADC readings
  temperature_batch samples{}; // creates array for converted temperature samples

  while (true) {
    if (xSemaphoreTake(sample_request, portMAX_DELAY) != pdTRUE) { // sampler sleep until the flag is raised, then lower it
    // portMAX_DELAY = time out that means "wait forever"
      continue; // jumps back to top of loop
    }

    adc.read(raw); // takes 64 ADC samples in 1 DMA transfer
    for (std::size_t i = 0; i < samples_per_batch; ++i) { // loop over all 64 samples
      samples[i] = to_temperature(raw[i]); // converts each raw sample to tenths of degree
    }

    // copies the 64 temperature samples into the queue for transmit_task (waits if the queue is full)
    xQueueSend(batch_queue, &samples, portMAX_DELAY);
  } // end of while (true)
} // end of sampler_task

struct transmit_task_parameters // bundles 3 objects transmit task need into one parameter 
{
  /// UART to the display board. transmit_task only writes to it.
  lab6::serial* serial;
  /// Times the two CRC implementations.
  lab2::steady_clock* clock;
  /// Sensor board's S1 (P1.1), configured with a pull-up: level() is false
  /// while held, and holding it corrupts the packet.
  lab1::input_pin* corruption_button;
};

/**
 * @brief Compute a CRC over the data and measure how long it took.
 *
 * Resets p_crc, feeds it p_data, and reads the checksum, all between two
 * clock readings. Afterward, p_crc.checksum() still returns the result.
 *
 * @return std::uint64_t - elapsed time in p_clock ticks
 */

 // CRC benchmark helpers 
std::uint64_t time_crc(lab6::crc32& p_crc,
                       std::span<std::uint8_t const> p_data,
                       lab2::steady_clock& p_clock)
{
  // Keep other tasks from running mid-measurement; interrupts still run,
  // so expect small variation between batches.
  vTaskSuspendAll();
  auto const start = p_clock.uptime();
  p_crc.reset();
  p_crc.update(p_data);
  // Read the result inside the timed region so the measurement includes it.
  [[maybe_unused]] auto const checksum = p_crc.checksum();
  auto const end = p_clock.uptime();
  xTaskResumeAll();
  return end - start;
}

unsigned long to_microseconds(std::uint64_t p_ticks, std::uint32_t p_frequency)
{
  return static_cast<unsigned long>(p_ticks * 1'000'000U / p_frequency);
}

// Waits for a batch, computes and benchmarks both CRCs, and sends the packet.
// Only writes to the serial port.
void transmit_task(void* p_parameters) // waits for a batch, computes and times both CRCs, then sends samples + CRC over UART
{
  auto const& parameters =
    *static_cast<transmit_task_parameters const*>(p_parameters); // turns the void* back into the struct holding the UART, clock and button

  auto& serial = *parameters.serial; // sets UART from struct to send packets
  auto& clock = *parameters.clock; // gets clock for strcut to time both CRC's
  auto& corruption_button = *parameters.corruption_button; // Gets S1 button from struct to corrupt packet when held

  // Only this task uses the CRCs, so they live on its stack.
  msp432_crc32_hardware hardware_crc;
  crc32_software software_crc;

  temperature_batch samples{}; // buffer for 1 recieved batch
// views the 64 samples as 128 raw bytes so the CRC and UART can use them
  std::span<std::uint8_t const> const sample_view(
    reinterpret_cast<std::uint8_t const*>(samples.data()), sizeof(samples));

  while (true) {// loop forever, one batch per pass
    if (xQueueReceive(batch_queue, &samples, portMAX_DELAY) != pdPASS) { // sleep until a batch is in the queue, then copy it into samples
      continue; // recieve failed and go back & wait 
    }

    // 2. Compute the CRC both ways, time each, and compare.
    auto const hardware_ticks = time_crc(hardware_crc, sample_view, clock);
    auto const hardware_checksum = hardware_crc.checksum();
    auto const software_ticks = time_crc(software_crc, sample_view, clock);
    auto const software_checksum = software_crc.checksum();

    auto const frequency = clock.frequency();
    debug_printf("CRC hardware: 0x%08lx in %lu ticks (%lu us)\n",
                 static_cast<unsigned long>(hardware_checksum),
                 static_cast<unsigned long>(hardware_ticks),
                 to_microseconds(hardware_ticks, frequency));
    debug_printf("CRC software: 0x%08lx in %lu ticks (%lu us) - %s\n",
                 static_cast<unsigned long>(software_checksum),
                 static_cast<unsigned long>(software_ticks),
                 to_microseconds(software_ticks, frequency),
                 hardware_checksum == software_checksum ? "match"
                                                        : "MISMATCH");
    {
      std::int32_t sum = 0;
      for (auto const sample : samples) {
        sum += sample;
      }
      auto const first = to_printable(samples.front());
      auto const average =
        to_printable(sum / static_cast<std::int32_t>(samples_per_batch));
      debug_printf("batch: first = %s%ld.%ld degC, average of %u = %s%ld.%ld "
                   "degC\n",
                   first.sign,
                   first.whole,
                   first.tenth,
                   static_cast<unsigned>(samples_per_batch),
                   average.sign,
                   average.whole,
                   average.tenth);
    }

    // 3. The packet uses the software CRC. Corrupt the data only after the
    //    CRC is saved, so the display board sees a mismatch.
    std::uint32_t const packet_crc = software_checksum;
    if (!corruption_button.level()) {
      samples[0] ^= 0b1;
      debug_printf("S1 held: flipped bit 0 of sample 0\n");
    }

    // 4. Send the samples, then the CRC, least significant byte first.
    if (!loopback_test) {
      serial.write(sample_view);
      std::array<std::uint8_t, crc_size> const crc_bytes{
        static_cast<std::uint8_t>(packet_crc),
        static_cast<std::uint8_t>(packet_crc >> 8),
        static_cast<std::uint8_t>(packet_crc >> 16),
        static_cast<std::uint8_t>(packet_crc >> 24),
      };
      serial.write(crc_bytes);
    }

    debug_batches_sent = debug_batches_sent + 1; // count batch as sent
  }
}

// TEST ONLY: with J1.4 jumpered to J1.3, sends a request byte every 2 s so
// the whole sensor pipeline can be exercised on one board. Created only when
// loopback_test is true.
void loopback_request_task(void* p_parameter) // function that pretents to be display board 
{
  auto& port = *static_cast<lab6::serial*>(p_parameter); // turns void* parameter back into UART objecct
  std::array<std::uint8_t, 1> const command{ sample_request_command }; // 1 byte message holding 0xA5 to send as fake request

  while (true) {// loop forever, sending one fake request every 2s
    vTaskDelay(pdMS_TO_TICKS(2000)); // sleep for 2s
    debug_printf("loopback: sending request\n"); // print that request has been sent
    port.write(command); // send 0xA5 out TX, which loops back to RX through the jumper
  } // end of while (true)
} // end of loopback_request_task

// =============================================================================
// Display board: task
// =============================================================================

// Sends a request on S1/S2, waits for the packet, checks its CRC, and sets
// the RGB LED from the average temperature.
void display_task(void* p_parameter)
{
  auto& serial = *static_cast<lab6::serial*>(p_parameter);
  crc32_software crc;

  msp432_input_pin s1(GPIO_PORT_P1, GPIO_PIN1);
  msp432_input_pin s2(GPIO_PORT_P1, GPIO_PIN4);
  msp432_output_pin red(GPIO_PORT_P2, GPIO_PIN0);
  msp432_output_pin green(GPIO_PORT_P2, GPIO_PIN1);
  msp432_output_pin blue(GPIO_PORT_P2, GPIO_PIN2);

  auto set_color = [&](bool p_red, bool p_green, bool p_blue) {
    red.level(p_red);
    green.level(p_green);
    blue.level(p_blue);
  };

  bool was_pressed = false;

  while (true) {
    // 1. Wait for a NEW press of S1 or S2 (pressed reads LOW), so holding a
    //    button sends only one request. The delay lets the blinker run.
    bool const pressed = !s1.level() || !s2.level();
    bool const new_press = pressed && !was_pressed;
    was_pressed = pressed;
    if (!new_press) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // 2. Note where the receive buffer is now, so only the reply counts,
    //    then send the request byte.
    auto const buffer = serial.receive_buffer();
    std::size_t read_cursor = serial.receive_cursor();
    std::array<std::uint8_t, 1> const request{ sample_request_command };
    serial.write(request);

    // 3. Copy 132 bytes out of the circular buffer, or stop at the timeout.
    std::array<std::uint8_t, packet_size> packet{};
    std::size_t received = 0;
    TickType_t const start = xTaskGetTickCount();

    while (received < packet_size &&
           (xTaskGetTickCount() - start) < reply_timeout) {
      if (read_cursor == serial.receive_cursor()) {
        vTaskDelay(1);  // no new byte yet, let other tasks run
        continue;
      }
      packet[received] = buffer[read_cursor];
      received++;
      read_cursor = (read_cursor + 1) % buffer.size();  // wrap around
    }

    if (received < packet_size) {
      debug_printf("timeout: got %u of %u bytes\n",
                   static_cast<unsigned>(received),
                   static_cast<unsigned>(packet_size));
      continue;
    }

    // 4. Split the packet into the 64 samples and the CRC that came with it
    //    (least significant byte first).
    temperature_batch samples{};
    std::memcpy(samples.data(), packet.data(), sample_bytes);
    std::uint32_t const received_crc =
      static_cast<std::uint32_t>(packet[sample_bytes]) |
      static_cast<std::uint32_t>(packet[sample_bytes + 1]) << 8 |
      static_cast<std::uint32_t>(packet[sample_bytes + 2]) << 16 |
      static_cast<std::uint32_t>(packet[sample_bytes + 3]) << 24;

    // 5. Recompute the CRC over the sample bytes. A mismatch means the data
    //    was corrupted, so drop it and show magenta.
    crc.reset();
    crc.update(std::span<std::uint8_t const>(packet.data(), sample_bytes));
    if (crc.checksum() != received_crc) {
      debug_printf("bad CRC - packet dropped\n");
      set_color(true, false, true);  // magenta
      continue;
    }

    // 6. Average the samples and pick the LED color.
    std::int32_t sum = 0;
    for (auto const sample : samples) {
      sum += sample;
    }
    std::int32_t const average = sum / std::int32_t{ samples_per_batch };
    auto const shown = to_printable(average);
    debug_printf("average = %s%ld.%ld degC\n",
                 shown.sign,
                 shown.whole,
                 shown.tenth);

    if (average < warm_tenths) {
      set_color(false, false, true);  // blue: cold
    } else if (average < hot_tenths) {
      set_color(true, true, false);  // yellow: warm
    } else {
      set_color(true, false, false);  // red: hot
    }
  }
}

/// Toggles the red LED on P1.0 every 250 ms, so you can see FreeRTOS is still
/// scheduling while the other tasks wait.
void blinker_task(void*)
{
  msp432_output_pin led(GPIO_PORT_P1, GPIO_PIN0);

  while (true) {
    led.level(!led.level());
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// =============================================================================
// Task storage
// =============================================================================

// Each task has its own stack and TCB. 512 words or more where semihosting
// printf runs (newlib printf is stack-hungry). Raise these if a task trips
// vApplicationStackOverflowHook().
constexpr std::uint32_t blinker_stack_words = 256;  // blinker task gets 256 word (1 KB) stack
constexpr std::uint32_t request_stack_words = 512;  // request task gets 512 word (2 KB) stack
constexpr std::uint32_t sampler_stack_words = 512;  // sampler task gets 512 word (2 KB) stack
constexpr std::uint32_t transmit_stack_words = 768; // transmit task gets 768 word (3 KB) stack
constexpr std::uint32_t display_stack_words = 768;  // display task gets 768 word (3 KB) stack

StaticTask_t blinker_tcb; // task control block: FreeRTOS's info about the blinker task (state, priority, etc.)
StackType_t blinker_stack[blinker_stack_words]; // blinker task's stack, sized by blinker_stack_words (256 words)

// Only one role's tasks are ever created, but both sets of storage are
// declared so either role compiles from this one file.
StaticTask_t request_tcb; // request task's control block
StackType_t request_stack[request_stack_words]; // request task's stack
StaticTask_t sampler_tcb; // sampler task's control block
StackType_t sampler_stack[sampler_stack_words]; // sampler task's stack
StaticTask_t transmit_tcb; // transmit task's control block
StackType_t transmit_stack[transmit_stack_words]; // transmit task's stack


StaticTask_t display_tcb; // display task's control block
StackType_t display_stack[display_stack_words]; // display task's stack

constexpr std::uint32_t loopback_stack_words = 512; // loopback task gets 512 word (2 KB) stack
StaticTask_t loopback_tcb; // loopback task's control block
StackType_t loopback_stack[loopback_stack_words]; // loopback task's stack
}  // namespace                                   


// DMA channel 7 has filled the ADC buffer: wake the task asleep in
// msp432_adc_driver::read(). Interrupt handlers must have external linkage,
// so this lives outside the unnamed namespace.
extern "C" void DMA_INT1_IRQHandler()
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  // A finished transfer leaves the channel's mode as "stop".
  bool const transfer_done =
    DMA_getChannelMode(UDMA_PRI_SELECT | DMA_CH7_ADC14) == UDMA_MODE_STOP;
  DMA_clearInterruptFlag(adc_dma_channel);

  TaskHandle_t const waiting_task = adc_waiting_task;
  if (transfer_done && waiting_task != nullptr) {
    vTaskNotifyGiveFromISR(waiting_task, &higher_priority_task_woken);
  }

  // Run the woken task as soon as this handler returns.
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

int main()
{
  WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

  if (debugger_connected()) { // only set up printing if a debugger is attached
    initialise_monitor_handles(); // connect printf to the CCS console (semihosting)
    std::setvbuf(stdout, // give printf output its own buffer
                 semihosting_stdout_buffer.data(), // use this 256 byte array as the buffer
                 _IOLBF, // send text when line ends
                 semihosting_stdout_buffer.size()); // how much room buffer has
    semihosting_ready = true; // printing is set up so debugger can print
  }

  // Both boards talk over eUSCI_A2 at 9600 baud. Static storage: main()'s
  // stack is reused for interrupts once the scheduler starts, so plain
  // locals would be overwritten.
  lab6::serial::settings uart_settings{}; // UART settings, starting from the defaults (115200 baud, no parity, 1 stop bit)
  uart_settings.baud_rate = 9'600;
  static msp432_serial_driver uart(system_clock_hz, uart_settings); // creating UART driver on P3.2/P3.3
  configASSERT(uart.ready()); // STOP if UART failed to set up
  // Pass the base-class pointer so the void* round trip in each task is exact.
  static lab6::serial* const serial_interface = &uart; // pointer to UART that gets passed to the tasks

  // LED1 blinking shows that FreeRTOS is scheduling.
  configASSERT(xTaskCreateStatic(blinker_task,
                                 "blinker",
                                 blinker_stack_words,
                                 nullptr,
                                 tskIDLE_PRIORITY + 1,
                                 blinker_stack,
                                 &blinker_tcb) != nullptr);

  if constexpr (this_device == device_role::sensor) {
    debug_printf("Lab 5 sensor board starting\n");


    // Synchronization objects must exist before any task that uses them runs.
    sample_request = xSemaphoreCreateBinaryStatic(&sample_request_storage); // creating sample request semaphore in static memory
    configASSERT(sample_request != nullptr);  // created empty (count 0)

// creating batch queue

    batch_queue = xQueueCreateStatic(batch_queue_length, // queue holds 1 batch
                                     sizeof(temperature_batch), // each item is 128 bytes (64 samples)
                                     batch_queue_storage.data(), // memory that holds the batch data
                                     &batch_queue_control); // memory for FreeRTOS's queue memory
    configASSERT(batch_queue != nullptr); // STOP if queue wan't created

    static msp432_adc_driver adc;
    static lab5::adc16_stream* const adc_interface = &adc;

    static msp432_steady_clock clock(system_clock_hz);
    static msp432_input_pin corruption_button(GPIO_PORT_P1, GPIO_PIN1);
    static transmit_task_parameters const transmit_parameters{
      .serial = serial_interface,
      .clock = &clock,
      .corruption_button = &corruption_button,
    };

    // Priorities: request > sampler > transmit. transmit busy-waits on UART TX
    // (~140 ms per packet), so it's lowest and the others can preempt it.
    configASSERT(xTaskCreateStatic(request_task, // create request task (stop if fails)
                                   "request", // task name shown in debugger 
                                   request_stack_words, // stack size (512 words)
                                   serial_interface, // UART parameter
                                   tskIDLE_PRIORITY + 3, // priority level of the 3 tasks (high)
                                   request_stack, // memory for its stack
                                   &request_tcb) != nullptr); // memory for its control block
    configASSERT(xTaskCreateStatic(sampler_task, // create sampler task (stop if fails)
                                   "sampler", // task name shown in debugger
                                   sampler_stack_words, // stack size (512 words)
                                   adc_interface, // ADC parameter
                                   tskIDLE_PRIORITY + 2, // priority level of the 3 tasks (middle)
                                   sampler_stack, // memory for its stack
                                   &sampler_tcb) != nullptr); // memory for its control block
    configASSERT(xTaskCreateStatic(
                   transmit_task, // create transmit task (stop if fails)
                   "transmit", // task name shown in debugger
                   transmit_stack_words, // stack size (768 words)
                   const_cast<transmit_task_parameters*>(&transmit_parameters), // struct parameter (UART, clock, button)
                   tskIDLE_PRIORITY + 1, // priority level of the 3 tasks (low)
                   transmit_stack, // memory for its stack
                   &transmit_tcb) != nullptr); // memory for its control block

    if (loopback_test) { // only create in single-board test mode
      configASSERT(xTaskCreateStatic(loopback_request_task, // create loopback task (stop if fails)
                                     "loopback", // task name shown in debugger
                                     loopback_stack_words, // stack size (512 words)
                                     serial_interface, // UART parameter
                                     tskIDLE_PRIORITY + 1, // priority level (low, same as transmit)
                                     loopback_stack, // memory for its stack
                                     &loopback_tcb) != nullptr); // memory for its control block
    }
  } else {
    debug_printf("Lab 5 display board starting\n");

    configASSERT(xTaskCreateStatic(display_task,
                                   "display",
                                   display_stack_words,
                                   serial_interface,
                                   tskIDLE_PRIORITY + 2,
                                   display_stack,
                                   &display_tcb) != nullptr);
  }

  vTaskStartScheduler();

  // Only reached if vTaskStartScheduler() itself fails to start.
  configASSERT(false);
  while (true) {
  }
}

// FreeRTOS uses these buffers for its idle and timer service tasks.
extern "C" void vApplicationGetIdleTaskMemory(
  StaticTask_t** ppxIdleTaskTCBBuffer,
  StackType_t** ppxIdleTaskStackBuffer,
  uint32_t* pulIdleTaskStackSize)
{
  static StaticTask_t idle_task_tcb;
  static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &idle_task_tcb;
  *ppxIdleTaskStackBuffer = idle_task_stack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

extern "C" void vApplicationGetTimerTaskMemory(
  StaticTask_t** ppxTimerTaskTCBBuffer,
  StackType_t** ppxTimerTaskStackBuffer,
  uint32_t* pulTimerTaskStackSize)
{
  static StaticTask_t timer_task_tcb;
  static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &timer_task_tcb;
  *ppxTimerTaskStackBuffer = timer_task_stack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

extern "C" void vApplicationMallocFailedHook(void)
{
  while (true) {
  }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char*)
{
  while (true) {
  }
}

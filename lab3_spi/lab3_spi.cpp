#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <cstdio>

#include <array>
#include <cstdint>
#include <cstring>

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"
#include "lab3_drivers.hpp"

extern "C" void initialise_monitor_handles(void);

class flash_memory
{
public:
  struct jedec_id
  {
    std::uint8_t manufacturer;
    std::uint8_t memory_type;
    std::uint8_t capacity_code;
  };

  static constexpr std::uint32_t block_size = 32;

  flash_memory(lab3::spi* p_spi, lab1::output_pin* p_chip_select)
    : m_spi(p_spi)
    , m_chip_select(p_chip_select)
  {
  }

  jedec_id read_jedec_id()
  {
    std::array<std::uint8_t, 1> const command{ cmd_jedec_id };
    std::array<std::uint8_t, 3> response{};

    select();
    m_spi->transfer(command, std::span<std::uint8_t>{});
    m_spi->transfer(std::span<std::uint8_t const>{}, response);
    deselect();

    return { response[0], response[1], response[2] };
  }

  std::uint32_t capacity_bytes()
  {
    auto const id = read_jedec_id();
    return std::uint32_t{ 1 } << id.capacity_code;
  }

  void read_block(std::uint32_t p_address, std::span<std::uint8_t> p_out)
  {
    std::array<std::uint8_t, 4> const command{
      cmd_read_data,
      static_cast<std::uint8_t>(p_address >> 16),
      static_cast<std::uint8_t>(p_address >> 8),
      static_cast<std::uint8_t>(p_address),
    };

    select();
    m_spi->transfer(command, std::span<std::uint8_t>{});
    m_spi->transfer(std::span<std::uint8_t const>{}, p_out);
    deselect();
  }

  void write_block(std::uint32_t p_address, std::span<std::uint8_t const> p_in)
  {
    write_enable();
    sector_erase(p_address);
    wait_until_ready();

    write_enable();
    page_program(p_address, p_in);
    wait_until_ready();
  }

private:
  static constexpr std::uint8_t cmd_write_enable = 0x06;
  static constexpr std::uint8_t cmd_read_status = 0x05;
  static constexpr std::uint8_t cmd_read_data = 0x03;
  static constexpr std::uint8_t cmd_page_program = 0x02;
  static constexpr std::uint8_t cmd_sector_erase = 0x20;
  static constexpr std::uint8_t cmd_jedec_id = 0x9F;

  static constexpr std::uint8_t status_busy = 1 << 0;

  void select()
  {
    m_chip_select->level(false);
  }

  void deselect()
  {
    m_chip_select->level(true);
  }

  void write_enable()
  {
    std::array<std::uint8_t, 1> const command{ cmd_write_enable };

    select();
    m_spi->transfer(command, std::span<std::uint8_t>{});
    deselect();
  }

  void sector_erase(std::uint32_t p_address)
  {
    std::array<std::uint8_t, 4> const command{
      cmd_sector_erase,
      static_cast<std::uint8_t>(p_address >> 16),
      static_cast<std::uint8_t>(p_address >> 8),
      static_cast<std::uint8_t>(p_address),
    };

    select();
    m_spi->transfer(command, std::span<std::uint8_t>{});
    deselect();
  }

  void page_program(std::uint32_t p_address, std::span<std::uint8_t const> p_in)
  {
    std::array<std::uint8_t, 4> const command{
      cmd_page_program,
      static_cast<std::uint8_t>(p_address >> 16),
      static_cast<std::uint8_t>(p_address >> 8),
      static_cast<std::uint8_t>(p_address),
    };

    select();
    m_spi->transfer(command, std::span<std::uint8_t>{});
    m_spi->transfer(p_in, std::span<std::uint8_t>{});
    deselect();
  }

  std::uint8_t read_status()
  {
    std::array<std::uint8_t, 1> const command{ cmd_read_status };
    std::array<std::uint8_t, 1> response{};

    select();
    m_spi->transfer(command, std::span<std::uint8_t>{});
    m_spi->transfer(std::span<std::uint8_t const>{}, response);
    deselect();

    return response[0];
  }

  void wait_until_ready()
  {
    while ((read_status() & status_busy) != 0) {
      continue;
    }
  }

  lab3::spi* m_spi = nullptr;
  lab1::output_pin* m_chip_select = nullptr;
};

namespace {

void print_block(std::span<std::uint8_t const> p_block)
{
  for (std::size_t i = 0; i < p_block.size(); i++) {
    std::printf("%02X ", static_cast<unsigned>(p_block[i]));
    if ((i % 16) == 15) {
      std::printf("\n");
    }
  }
  if ((p_block.size() % 16) != 0) {
    std::printf("\n");
  }

  std::printf("ascii: \"");
  for (auto const byte : p_block) {
    std::printf("%c", (byte >= 0x20 && byte < 0x7F) ? byte : '.');
  }
  std::printf("\"\n");
}

}  // namespace

int main()
{
  MAP_WDT_A_holdTimer();

  initialise_monitor_handles();

  gpio_output chip_select{ 5, 0 };
  chip_select.configure(lab1::output_pin::settings{});
  chip_select.level(true);

  spi_b0 spi;
  spi.configure(lab3::spi::settings{
    .clock_rate = 1'000'000,
    .bus_mode = lab3::spi::mode::m0,
  });

  flash_memory flash{ &spi, &chip_select };

  auto const id = flash.read_jedec_id();
  auto const capacity = flash.capacity_bytes();

  std::printf("manufacturer ID: 0x%02X\n", static_cast<unsigned>(id.manufacturer));
  std::printf("memory type:     0x%02X\n", static_cast<unsigned>(id.memory_type));
  std::printf("capacity code:   0x%02X\n", static_cast<unsigned>(id.capacity_code));
  std::printf("capacity:        %u bytes (%u Mbit)\n",
              static_cast<unsigned>(capacity),
              static_cast<unsigned>(capacity / 131072));

  std::array<std::uint8_t, flash_memory::block_size> block{};

  flash.read_block(0, block);
  std::printf("\nfirst %u bytes before write:\n",
              static_cast<unsigned>(block.size()));
  print_block(block);

  std::printf("\nenter up to %u bytes to write: ",
              static_cast<unsigned>(block.size()));
  std::fflush(stdout);

  std::array<char, 128> line{};
  if (std::fgets(line.data(), static_cast<int>(line.size()), stdin) == nullptr) {
    std::printf("\nno input, skipping write\n");
    while (true) {
      continue;
    }
  }

  std::array<std::uint8_t, flash_memory::block_size> payload{};
  std::size_t length = 0;

  for (std::size_t i = 0; i < line.size() && line[i] != '\0'; i++) {
    auto const character = line[i];

    if (character == '\n' || character == '\r') {
      break;
    }

    if (character == 0x08 || character == 0x7F) {
      if (length > 0) {
        length--;
        payload[length] = 0;
      }
      continue;
    }

    if (character < 0x20) {
      continue;
    }

    if (length < payload.size()) {
      payload[length] = static_cast<std::uint8_t>(character);
      length++;
    }
  }

  std::printf("writing %u bytes...\n", static_cast<unsigned>(length));
  flash.write_block(0, payload);

  flash.read_block(0, block);
  std::printf("\nfirst %u bytes after write:\n",
              static_cast<unsigned>(block.size()));
  print_block(block);

  while (true) {
    continue;
  }
}

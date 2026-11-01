// Minimal Feetech STS (ST3215) bus driver: the C++ counterpart of tools/feetech_bus.py,
// plus SYNC READ / SYNC WRITE for one bus cycle per control period.
//
// Protocol (half duplex, 1 Mbps, multi-byte values little-endian):
//   instruction: FF FF ID LEN INSTR PARAM... CHK
//   status:      FF FF ID LEN ERR   PARAM... CHK
//   LEN = number of params + 2, CHK = ~(ID + LEN + INSTR/ERR + sum(params)) & 0xFF
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace so101_hardware
{

// Control table (STS3215). EEPROM is below address 40, RAM from 40 on. With the EEPROM lock
// on (address 55 = 1, the default) a write to an EEPROM address only changes the RAM copy:
// it takes effect at once and is gone after a power cycle.
constexpr uint8_t P_COEFFICIENT = 21;     // u8, position loop P (factory 32)
constexpr uint8_t D_COEFFICIENT = 22;     // u8
constexpr uint8_t I_COEFFICIENT = 23;     // u8
constexpr uint8_t HOMING_OFFSET = 31;     // u16, sign-magnitude (bit 11)
constexpr uint8_t TORQUE_ENABLE = 40;     // u8
constexpr uint8_t ACCELERATION = 41;      // u8, unit 100 step/s^2, 0 = no limit
constexpr uint8_t GOAL_POSITION = 42;     // u16
constexpr uint8_t GOAL_SPEED = 46;        // u16, unit step/s, 0 = no limit
constexpr uint8_t TORQUE_LIMIT = 48;      // u16, unit 0.1 %
constexpr uint8_t PRESENT_POSITION = 56;  // u16, sign bit 15, followed by PRESENT_SPEED (u16, sign bit 15)

constexpr int TICKS_PER_TURN = 4096;

class FeetechError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

int decode_sm(uint16_t value, int sign_bit);

class FeetechBus
{
public:
  // Opens the port with an exclusive flock, like the Python tools: only one owner of the bus.
  explicit FeetechBus(const std::string & port, int timeout_ms = 20);
  ~FeetechBus();
  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

  bool ping(uint8_t id);
  std::vector<uint8_t> read(uint8_t id, uint8_t addr, uint8_t size);
  void write(uint8_t id, uint8_t addr, const std::vector<uint8_t> & data);
  uint16_t read_u16(uint8_t id, uint8_t addr);
  void write_u8(uint8_t id, uint8_t addr, uint8_t value);
  void write_u16(uint8_t id, uint8_t addr, uint16_t value);

  // SYNC READ: `size` bytes from `addr` of every servo, one reply each, in order.
  std::vector<std::vector<uint8_t>> sync_read(const std::vector<uint8_t> & ids, uint8_t addr, uint8_t size);
  // SYNC WRITE (broadcast, no reply): data[i] goes to ids[i].
  void sync_write(const std::vector<uint8_t> & ids, uint8_t addr, const std::vector<std::vector<uint8_t>> & data);

  // The single /dev/serial/by-id/usb-1a86_* adapter (the Waveshare board's CH343).
  static std::string find_port();

private:
  void send(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params);
  std::vector<uint8_t> receive(uint8_t id);
  std::vector<uint8_t> transact(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params);

  int fd_ = -1;
  int timeout_ms_;
  std::vector<uint8_t> buf_;
  std::vector<uint8_t> last_sent_;
};

}  // namespace so101_hardware

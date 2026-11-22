#include "so101_hardware/feetech_bus.hpp"

#include <asm/termbits.h>
#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <numeric>

namespace so101_hardware
{

namespace
{
constexpr uint8_t PING = 0x01;
constexpr uint8_t READ = 0x02;
constexpr uint8_t WRITE = 0x03;
constexpr uint8_t SYNC_READ = 0x82;
constexpr uint8_t SYNC_WRITE = 0x83;
constexpr uint8_t BROADCAST = 0xFE;
constexpr int BAUDRATE = 1000000;

uint8_t checksum(const uint8_t * begin, const uint8_t * end)
{
  return static_cast<uint8_t>(~std::accumulate(begin, end, 0u) & 0xFF);
}
}  // namespace

int decode_sm(uint16_t value, int sign_bit)
{
  const int magnitude = value & ((1 << sign_bit) - 1);
  return (value & (1 << sign_bit)) ? -magnitude : magnitude;
}

FeetechBus::FeetechBus(const std::string & port, int timeout_ms)
: timeout_ms_(timeout_ms)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw FeetechError(port + ": " + std::strerror(errno));
  }
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd_);
    throw FeetechError(port + " is in use by another tool (joint_states, teleop, ...)");
  }
  // termios2 for the non-standard 1 Mbps rate, raw 8N1
  struct termios2 tio {};
  if (::ioctl(fd_, TCGETS2, &tio) != 0) {
    ::close(fd_);
    throw FeetechError(port + ": TCGETS2 failed");
  }
  tio.c_cflag = CS8 | CREAD | CLOCAL | BOTHER;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_ispeed = tio.c_ospeed = BAUDRATE;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (::ioctl(fd_, TCSETS2, &tio) != 0) {
    ::close(fd_);
    throw FeetechError(port + ": TCSETS2 failed");
  }
}

FeetechBus::~FeetechBus()
{
  if (fd_ >= 0) {
    ::close(fd_);  // also releases the flock
  }
}

std::string FeetechBus::find_port()
{
  glob_t g{};
  ::glob("/dev/serial/by-id/usb-1a86_*", 0, nullptr, &g);
  std::vector<std::string> hits(g.gl_pathv, g.gl_pathv + g.gl_pathc);
  ::globfree(&g);
  if (hits.size() == 1) {
    return hits[0];
  }
  throw FeetechError(hits.empty() ? "no servo adapter found (/dev/serial/by-id/usb-1a86_*)"
                                  : "several WCH adapters found, set the port parameter");
}

void FeetechBus::send(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params)
{
  std::vector<uint8_t> packet{0xFF, 0xFF, id, static_cast<uint8_t>(params.size() + 2), instr};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(checksum(packet.data() + 2, packet.data() + packet.size()));
  ::ioctl(fd_, TCFLSH, TCIFLUSH);  // drop stale bytes from an earlier, timed-out reply
  buf_.clear();
  if (::write(fd_, packet.data(), packet.size()) != static_cast<ssize_t>(packet.size())) {
    throw FeetechError("write failed: " + std::string(std::strerror(errno)));
  }
  last_sent_ = packet;
}

std::vector<uint8_t> FeetechBus::receive(uint8_t id)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
  while (true) {
    // parse every complete frame that is buffered
    static const uint8_t header[] = {0xFF, 0xFF};
    while (true) {
      const auto start = std::search(buf_.begin(), buf_.end(), header, header + 2);
      if (buf_.end() - start < 4) {
        break;
      }
      const size_t frame_len = 4 + start[3];
      if (static_cast<size_t>(buf_.end() - start) < frame_len) {
        break;
      }
      std::vector<uint8_t> frame(start, start + frame_len);
      buf_.erase(buf_.begin(), start + frame_len);
      if (frame == last_sent_) {
        continue;  // local echo on some adapters
      }
      if (checksum(frame.data() + 2, frame.data() + frame.size() - 1) != frame.back()) {
        throw FeetechError("id " + std::to_string(id) + ": bad checksum");
      }
      if (frame[2] == id) {
        return std::vector<uint8_t>(frame.begin() + 5, frame.end() - 1);
      }
    }
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      throw FeetechError("id " + std::to_string(id) + ": no reply");
    }
    pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, static_cast<int>(left)) > 0) {
      uint8_t chunk[256];
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n > 0) {
        buf_.insert(buf_.end(), chunk, chunk + n);
      }
    }
  }
}

std::vector<uint8_t> FeetechBus::transact(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params)
{
  // one corrupted reply on a noisy bus is not fatal: try twice
  for (int attempt = 0;; ++attempt) {
    send(id, instr, params);
    try {
      return receive(id);
    } catch (const FeetechError &) {
      if (attempt == 1) {
        throw;
      }
    }
  }
}

bool FeetechBus::ping(uint8_t id)
{
  try {
    transact(id, PING, {});
    return true;
  } catch (const FeetechError &) {
    return false;
  }
}

std::vector<uint8_t> FeetechBus::read(uint8_t id, uint8_t addr, uint8_t size)
{
  auto data = transact(id, READ, {addr, size});
  if (data.size() != size) {
    throw FeetechError("id " + std::to_string(id) + ": short read");
  }
  return data;
}

void FeetechBus::write(uint8_t id, uint8_t addr, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params{addr};
  params.insert(params.end(), data.begin(), data.end());
  transact(id, WRITE, params);
}

uint16_t FeetechBus::read_u16(uint8_t id, uint8_t addr)
{
  const auto d = read(id, addr, 2);
  return static_cast<uint16_t>(d[0] | (d[1] << 8));
}

void FeetechBus::write_u8(uint8_t id, uint8_t addr, uint8_t value)
{
  write(id, addr, {value});
}

void FeetechBus::write_u16(uint8_t id, uint8_t addr, uint16_t value)
{
  write(id, addr, {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)});
}

std::vector<std::vector<uint8_t>> FeetechBus::sync_read(
  const std::vector<uint8_t> & ids, uint8_t addr, uint8_t size)
{
  std::vector<uint8_t> params{addr, size};
  params.insert(params.end(), ids.begin(), ids.end());
  send(BROADCAST, SYNC_READ, params);
  std::vector<std::vector<uint8_t>> out;
  for (const auto id : ids) {
    auto data = receive(id);
    if (data.size() != size) {
      throw FeetechError("id " + std::to_string(id) + ": short sync read");
    }
    out.push_back(std::move(data));
  }
  return out;
}

void FeetechBus::sync_write(
  const std::vector<uint8_t> & ids, uint8_t addr, const std::vector<std::vector<uint8_t>> & data)
{
  std::vector<uint8_t> params{addr, static_cast<uint8_t>(data.front().size())};
  for (size_t i = 0; i < ids.size(); ++i) {
    params.push_back(ids[i]);
    params.insert(params.end(), data[i].begin(), data[i].end());
  }
  send(BROADCAST, SYNC_WRITE, params);
}

}  // namespace so101_hardware

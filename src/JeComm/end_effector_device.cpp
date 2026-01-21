#include "end_effector_device.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>

EndEffectorDevice::EndEffectorDevice(const std::string& port, int baudrate, const SerialOptions& options)
    : port_(port),
      baudrate_(baudrate),
      options_(options),
      transport_(nullptr),
      last_reinit_ms_(-1)
{
}

bool EndEffectorDevice::Open()
{
    if (!transport_)
        transport_ = std::unique_ptr<ITransport>(new SerialTransport(port_, baudrate_, options_));

    if (!transport_->open())
    {
        transport_.reset();
        return false;
    }
    return true;
}

void EndEffectorDevice::Close()
{
    if (transport_)
        transport_->close();
}

int64_t EndEffectorDevice::NowSteadyMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

uint8_t EndEffectorDevice::ChecksumSum(const std::vector<uint8_t>& payload) const
{
    uint32_t sum = 0;
    for (uint8_t b : payload)
        sum += b;
    return static_cast<uint8_t>(sum & 0xFF);
}

bool EndEffectorDevice::SendFrame(const std::vector<uint8_t>& payload)
{
    if (!transport_)
    {
        std::cerr << "end effector serial not ready, skip command\n";
        return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 3);
    frame.push_back(0x55);
    frame.push_back(0xAA);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(ChecksumSum(payload));

    if (transport_->write(frame.data(), frame.size()) != static_cast<ssize_t>(frame.size()))
    {
        std::cerr << "end effector write failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool EndEffectorDevice::SendRawBytes(const std::vector<uint8_t>& bytes)
{
    if (!transport_)
    {
        std::cerr << "end effector serial not ready, skip raw command\n";
        return false;
    }

    if (bytes.empty())
        return true;

    if (transport_->write(bytes.data(), bytes.size()) != static_cast<ssize_t>(bytes.size()))
    {
        std::cerr << "end effector raw write failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

void EndEffectorDevice::StopAll()
{
    // 55 AA 05 06 01 02 03 04 05 06 20
    SendFrame({0x05, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06});
}

void EndEffectorDevice::StopMotor(uint8_t motor_id)
{
    // 55 AA 05 01 <ID> <checksum>
    SendFrame({0x05, 0x01, motor_id});
}

void EndEffectorDevice::TwoFinger(uint8_t percent)
{
    std::cout << "send percent: 0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(percent)
            << std::dec << std::endl;
    // 55 AA 09 <percent> 00 <checksum>
    SendFrame({0x09, percent, 0x00});
}

void EndEffectorDevice::ThreeFinger(uint8_t percent)
{
    std::cout << "send percent: 0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(percent)
            << std::dec << std::endl;
    // 55 AA 0A <percent> 00 <checksum>
    SendFrame({0x0A, percent, 0x00});
}

void EndEffectorDevice::SetPositions(uint8_t m1, uint8_t m2, uint8_t m3,
                                     uint8_t m4, uint8_t m5, uint16_t m6)
{
    // 55 AA 03 06 01 02 03 04 05 06 m1 m2 m3 m4 m5 m6_hi m6_lo <checksum>
    std::vector<uint8_t> payload = {
        0x03, 0x06, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
        m1, m2, m3, m4, m5,
        static_cast<uint8_t>((m6 >> 8) & 0xFF),
        static_cast<uint8_t>(m6 & 0xFF)
    };
    SendFrame(payload);
}

void EndEffectorDevice::Reinit()
{
    // 直接发送三个字节：02 FF 00
    // 若你的协议需要加 55 AA + checksum，请改为：
    // SendFrame({0x02, 0xFF, 0x00});
    SendRawBytes({0x02, 0xFF, 0x00});
}

void EndEffectorDevice::HandlePreset(int preset)
{
    switch (preset)
    {
        case 0:
            StopAll();
            break;
        case 1:
            StopMotor(0x01);
            break;
        case 2:
            StopMotor(0x02);
            break;
        case 3:
            StopMotor(0x03);
            break;
        case 4:
            StopMotor(0x04);
            break;
        case 5:
            StopMotor(0x05);
            break;
        case 6:
            StopMotor(0x06);
            break;
        case 10:
            TwoFinger(0x00);
            break;
        case 11:
            TwoFinger(0x47);
            break;
        case 20:
            ThreeFinger(0x00);
            break;
        case 21:
            ThreeFinger(0x58);
            break;
        case 99:
        {
            const int64_t now = NowSteadyMs();

            while (true)
            {
                int64_t last = last_reinit_ms_.load(std::memory_order_relaxed);

                // If executed once, ignore further commands within 2 seconds
                if (last >= 0 && (now - last) < 2000)
                {
                    std::cerr << "[end_effector] reinit preset=99 ignored (cooldown), dt_ms="
                            << (now - last) << "\n";
                    break;
                }

                // Try to "claim" this execution window
                if (last_reinit_ms_.compare_exchange_weak(
                        last, now,
                        std::memory_order_relaxed,
                        std::memory_order_relaxed))
                {
                    std::cerr << "[end_effector] reinit preset=99 execute, send 02 FF 00\n";
                    Reinit();
                    break;
                }

                // CAS failed: retry (another thread updated last)
            }
            break;
        }
        default:
            std::cerr << "unknown end effector preset: " << preset << "\n";
            break;
    }
}

void EndEffectorDevice::HandlePosition(double position)
{
    int pos = static_cast<int>(std::lround(position));
    if (pos < 0)
        pos = 0;
    if (pos > 2000)
        pos = 2000;

    // motor6 uses 0~2000, other motors keep at 0 by default
    SetPositions(0, 0, 0, 0, 0, static_cast<uint16_t>(pos));
}

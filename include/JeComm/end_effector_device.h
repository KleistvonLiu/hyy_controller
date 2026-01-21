#ifndef __end_effector_device__
#define __end_effector_device__

#include "dh_transport.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class EndEffectorDevice
{
public:
    EndEffectorDevice(const std::string& port, int baudrate, const SerialOptions& options);

    bool Open();
    void Close();

    void HandlePreset(int preset);
    void HandlePosition(double position);

private:
    uint8_t ChecksumSum(const std::vector<uint8_t>& payload) const;
    bool SendFrame(const std::vector<uint8_t>& payload);
    bool SendRawBytes(const std::vector<uint8_t>& bytes);

    void StopAll();
    void StopMotor(uint8_t motor_id);
    void TwoFinger(uint8_t percent);
    void ThreeFinger(uint8_t percent);
    void SetPositions(uint8_t m1, uint8_t m2, uint8_t m3,
                      uint8_t m4, uint8_t m5, uint16_t m6);
    void Reinit();

    static int64_t NowSteadyMs();

    std::string port_;
    int baudrate_;
    SerialOptions options_;
    std::unique_ptr<ITransport> transport_;
    std::atomic<int64_t> last_reinit_ms_;
};

#endif //__end_effector_device__

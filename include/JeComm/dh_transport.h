#ifndef __dh_transport__
#define __dh_transport__

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

struct SerialOptions
{
    int vmin;
    int vtime;
    enum SerialProfile
    {
        kProfileDhDevice = 0,
        kProfileJEServerLegacy = 1
    };
    SerialProfile profile;

    SerialOptions() : vmin(1), vtime(1), profile(kProfileDhDevice) {}
    SerialOptions(int vmin_in, int vtime_in, SerialProfile profile_in = kProfileDhDevice)
        : vmin(vmin_in), vtime(vtime_in), profile(profile_in) {}
};

class ITransport
{
public:
    virtual ~ITransport() {}

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual ssize_t write(const uint8_t* data, size_t len) = 0;
    virtual ssize_t read(uint8_t* data, size_t len) = 0;
    virtual int native_handle() const = 0;
};

class SerialTransport : public ITransport
{
public:
    SerialTransport(const std::string& port, int baudrate, const SerialOptions& options = SerialOptions());
    ~SerialTransport() override;

    bool open() override;
    void close() override;
    ssize_t write(const uint8_t* data, size_t len) override;
    ssize_t read(uint8_t* data, size_t len) override;
    int native_handle() const override;

private:
    std::string port_;
    int baudrate_;
    SerialOptions options_;
    int fd_;
};

class TcpTransport : public ITransport
{
public:
    explicit TcpTransport(const std::string& endpoint);
    ~TcpTransport() override;

    bool open() override;
    void close() override;
    ssize_t write(const uint8_t* data, size_t len) override;
    ssize_t read(uint8_t* data, size_t len) override;
    int native_handle() const override;

private:
    std::string endpoint_;
    int fd_;
};

#endif //__dh_transport__

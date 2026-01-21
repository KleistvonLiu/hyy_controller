#include "dh_transport.h"
#include "dh_device.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

static bool apply_serial_options(int fd, const SerialOptions& options)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
        return false;

    tty.c_cc[VMIN] = static_cast<cc_t>(options.vmin);
    tty.c_cc[VTIME] = static_cast<cc_t>(options.vtime);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        return false;

    return true;
}

static bool baudrate_to_speed(int baudrate, speed_t* speed)
{
    if (!speed)
        return false;
    switch (baudrate)
    {
        case 115200:
            *speed = B115200;
            return true;
        case 38400:
            *speed = B38400;
            return true;
        case 19200:
            *speed = B19200;
            return true;
        case 9600:
            *speed = B9600;
            return true;
        default:
            return false;
    }
}

static int open_jeserver_serial(const std::string& port, int baudrate, const SerialOptions& options)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
        return -1;

    struct termios tty;
    std::memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        close(fd);
        return -1;
    }

    speed_t speed;
    if (!baudrate_to_speed(baudrate, &speed))
    {
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = static_cast<cc_t>(options.vmin);
    tty.c_cc[VTIME] = static_cast<cc_t>(options.vtime);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        close(fd);
        return -1;
    }

    return fd;
}

SerialTransport::SerialTransport(const std::string& port, int baudrate, const SerialOptions& options)
    : port_(port), baudrate_(baudrate), options_(options), fd_(-1)
{
}

SerialTransport::~SerialTransport()
{
    close();
}

bool SerialTransport::open()
{
    if (fd_ >= 0)
        return true;
    if (options_.profile == SerialOptions::kProfileJEServerLegacy)
    {
        fd_ = open_jeserver_serial(port_, baudrate_, options_);
        return fd_ >= 0;
    }

    fd_ = serial_connect(port_, baudrate_);
    if (fd_ < 0)
        return false;
    if (!apply_serial_options(fd_, options_))
    {
        disconnect_device(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

void SerialTransport::close()
{
    if (fd_ >= 0)
    {
        disconnect_device(fd_);
        fd_ = -1;
    }
}

ssize_t SerialTransport::write(const uint8_t* data, size_t len)
{
    if (fd_ < 0)
        return -1;
    return device_write_all(fd_, data, len) ? static_cast<ssize_t>(len) : -1;
}

ssize_t SerialTransport::read(uint8_t* data, size_t len)
{
    if (fd_ < 0)
        return -1;
    return device_read(fd_, reinterpret_cast<char*>(data), static_cast<int>(len));
}

int SerialTransport::native_handle() const
{
    return fd_;
}

TcpTransport::TcpTransport(const std::string& endpoint)
    : endpoint_(endpoint), fd_(-1)
{
}

TcpTransport::~TcpTransport()
{
    close();
}

bool TcpTransport::open()
{
    if (fd_ >= 0)
        return true;
    fd_ = tcp_connect(endpoint_);
    return fd_ >= 0;
}

void TcpTransport::close()
{
    if (fd_ >= 0)
    {
        disconnect_device(fd_);
        fd_ = -1;
    }
}

ssize_t TcpTransport::write(const uint8_t* data, size_t len)
{
    if (fd_ < 0)
        return -1;
    return device_write_all(fd_, data, len) ? static_cast<ssize_t>(len) : -1;
}

ssize_t TcpTransport::read(uint8_t* data, size_t len)
{
    if (fd_ < 0)
        return -1;
    return device_read(fd_, reinterpret_cast<char*>(data), static_cast<int>(len));
}

int TcpTransport::native_handle() const
{
    return fd_;
}


#include "user/inspire.h"
// 寄存器字典
std::map<std::string, int> regdict = {
    {"ID", 1000},
    {"baudrate", 1001},
    {"clearErr", 1003},
    {"forceClb", 1008},
    {"angleSet", 1480},
    {"forceSet", 1486},
    {"speedSet", 1498},
    {"angleAct", 1510},
    {"forceAct", 1528},
    {"errCode", 1540},
    {"statusCode", 1546},
    {"temp", 1552},
    {"actionSeq", 2320},
    {"actionRun", 2322}
};

// 打开串口
int init_serial(const std::string& port, int baudrate) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "打开串口失败: " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr失败: " << strerror(errno) << std::endl;
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8位数据
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5; // 0.5秒超时

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr失败: " << strerror(errno) << std::endl;
        return -1;
    }

    // std::cout << "串口 " << port << " 已打开" << std::endl;
    return fd;
}

// 十进制地址转二进制字符串（写）
std::string convert_address_to_binary_write(int address_dec, const std::string& id_value) {
    std::string address_bin = std::bitset<16>(address_dec).to_string();
    auto pos = address_bin.find('1'); 
    address_bin=address_bin.substr(pos); 
    std::string id_bin = std::bitset<14>(std::stoi(id_value, nullptr, 2)).to_string();
    return "0000010" + address_bin + id_bin;
}

// 十进制地址转二进制字符串（读）
std::string convert_address_to_binary_read(int address_dec, const std::string& id_value) {
    std::string address_bin = std::bitset<16>(address_dec).to_string();
    std::string id_bin = std::bitset<14>(std::stoi(id_value, nullptr, 2)).to_string();
    return "0000000" + address_bin + id_bin;
}

// 二进制转十六进制
std::string binary_to_hex(const std::string& binary_string) {
    unsigned long decimal_value = std::stoul(binary_string, nullptr, 2);
    char buffer[20];
    sprintf(buffer, "%lX", decimal_value);
    return std::string(buffer);
}

// 数字转字节数组
std::vector<uint8_t> convert_number_to_bytes(unsigned long number) {
    std::vector<uint8_t> byte_array;
    while (number > 0) {
        byte_array.insert(byte_array.begin(), number & 0xFF);
        number >>= 8;
    }
    if (byte_array.empty()) byte_array.push_back(0);
    return byte_array;
}

// 写寄存器
void write_register(int fd, int address, const std::vector<int>& values, const std::string& id_value) {
    // 计算扩展标识符（写操作地址转换）
    std::string ext_id_bin = convert_address_to_binary_write(address, id_value);
    std::string ext_id_hex = binary_to_hex(ext_id_bin);
    unsigned long ext_id_number = std::stoul(ext_id_hex, nullptr, 16);

    // 转换成字节数组（扩展 ID 部分）
    std::vector<uint8_t> ext_id_bytes = convert_number_to_bytes(ext_id_number);

    // 构建发送缓冲区
    std::vector<uint8_t> send_buffer;

    // 添加扩展 ID 字节
    send_buffer.insert(send_buffer.end(), ext_id_bytes.begin(), ext_id_bytes.end());

    // 添加数据：低字节在前，高字节在后
    for (int value : values) {
        send_buffer.push_back(value & 0xFF);        // 低字节
        send_buffer.push_back((value >> 8) & 0xFF); // 高字节
    }

    // 计算数据长度（仅计算，不使用）
    // int data_length = values.size() * 2;

    // 发送数据
    ssize_t written = write(fd, send_buffer.data(), send_buffer.size());
    if (written < 0) {
        std::cerr << "写入失败: " << strerror(errno) << std::endl;
    }
    // for (size_t i=0;i<send_buffer.size();i++)
    // {
    //     printf("%X,",send_buffer[i]);
    // }
    // printf("\n");
    // 等待 50ms
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 清空串口输入缓冲区
    tcflush(fd, TCIFLUSH);
}

// 写6个数据
void write6(int fd, const std::string& reg_name, const std::vector<int>& val, const std::string& id_value) {
    //std::cout << "设置" << reg_name << "的值依次为：";
    // for (auto v : val) std::cout << v << " ";
    // std::cout << std::endl;

    if (regdict.find(reg_name) != regdict.end()) {
        write_register(fd, regdict[reg_name], val, id_value);
    }
}

// // 读寄存器
// std::vector<int> read_register(int fd, int address, const std::string& id_value) {
//     try {
//         std::string ext_id_bin = convert_address_to_binary_read(address, id_value);
//         std::string ext_id_hex = binary_to_hex(ext_id_bin);
//         unsigned long ext_id_number = std::stoul(ext_id_hex, nullptr, 16);

//         std::vector<uint8_t> send_buffer = convert_number_to_bytes(ext_id_number);
//         send_buffer.push_back(0x0C); // 数据长度

//         write(fd, send_buffer.data(), send_buffer.size());

//         std::this_thread::sleep_for(std::chrono::milliseconds(50));

//         uint8_t response[16];
//         int n = read(fd, response, sizeof(response));
//         if (n <= 0) {
//             std::cerr << "未收到任何响应或超时" << std::endl;
//             return {};
//         }
//         if (n < 16) {
//             std::cerr << "响应长度不足，忽略" << std::endl;
//             return {};
//         }

//         std::vector<int> values;
//         for (int i = 6; i < 16; i += 2) {
//             int low = response[i];
//             int high = response[i + 1];
//             int value = (high << 8) | low;
//             if (value > 30000) value -= 65535;
//             values.push_back(value);
//         }

//         tcflush(fd, TCIFLUSH);
//         return values;

//     } catch (std::exception& e) {
//         std::cerr << "读取寄存器发生异常: " << e.what() << std::endl;
//         return {};
//     }
// }

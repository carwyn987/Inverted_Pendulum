#include <iostream>
#include <fstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

int set_serial_attributes(int fd, int speed, int parity) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo,
                                                // no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 0;                         // read doesn't block
    tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls,
                                                // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return -1;
    }
    return 0;
}

enum InputType {
    NONE,
    X_POSITION,
    THETA
};

// Struct to hold the result of the serial read
struct SerialResult {
    InputType type;
    float value;
};

// Function to read floats from serial
SerialResult read_from_serial(int fd) {
    char buf;
    std::string data;
    SerialResult result = {NONE, 0.0};

    while (true) {
        int n = read(fd, &buf, 1);
        if (n > 0) {
            if (buf == '\n' || buf == '\r') {
                // Find the last 'x' or 'a' in the data
                int lastX = data.rfind('x');
                int lastT = data.rfind('a');
                int pos = std::max(lastX, lastT);

                if (pos >= 0) {
                    char type = data[pos];
                    std::string valueStr = data.substr(pos + 1);
                    try {
                        result.value = std::stof(valueStr);
                        result.type = (type == 'x') ? X_POSITION : THETA;
                    } catch (const std::invalid_argument& ia) {
                        // Ignore invalid argument and reset data
                        result.value = -1;
                    }
                }
                data.clear(); // Clear the buffer for the next message
                return result;
            } else {
                data += buf;
            }
        } else if (n < 0) {
            std::cerr << "Error from read: " << strerror(errno) << std::endl;
            throw std::runtime_error("Error reading from serial");
            break;
        }
    }
    return result;
}

void send_floats_to_serial(int fd, float value) {
    std::string data = std::to_string(value) + "\n";
    int n = write(fd, data.c_str(), data.size());
    if (n < 0) {
        std::cerr << "Error from write: " << strerror(errno) << std::endl;
    }
}
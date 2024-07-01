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


float read_floats_from_serial(int fd) {
    char buf;
    std::string data;
    float value = -9999.0;

    while (value == -9999.0) {
        int n = read(fd, &buf, 1);
        if (n > 0) {
            if (buf == '\n') {
                try {
                    value = std::stof(data);
                } catch (const std::invalid_argument& ia) {
                    // Ignore invalid argument and reset data
                }
                data.clear(); // Clear the buffer for the next message
            } else {
                data += buf;
            }
        } else if (n < 0) {
            std::cerr << "Error from read: " << strerror(errno) << std::endl;
            break;
        }
    }
    return value;
}
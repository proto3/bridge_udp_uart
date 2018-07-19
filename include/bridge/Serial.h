#ifndef SERIAL_H
#define SERIAL_H

#include <bridge/UDP.h>
#include <thread>
#include <string>

class UDP;

class Serial
{
public:
    Serial(std::string port, int baudrate);
    ~Serial();

    static bool is_valid_tty(const char* path);
    bool send(uint8_t *buffer, uint32_t length);
    void start();
    void join();

    void set_udp(UDP *udp);

private:
    UDP *m_udp;

    void read_loop();
    std::thread *reading_thread;
    int serial_fd;
};

#endif

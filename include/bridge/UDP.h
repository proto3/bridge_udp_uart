#ifndef UDP_H
#define UDP_H

#include <bridge/Serial.h>
#include <thread>
#include <string>
#include <arpa/inet.h>

class Serial;

class UDP
{
public:
    //Client constructor
    UDP(std::string target_ip, int target_port, int local_port);
    //Server constructor
    UDP(int local_port);
    ~UDP();

    static bool is_valid_ip(const char* ip);
    bool send(uint8_t *buffer, uint32_t length);
    void start();
    void join();

    void set_serial(Serial *serial);

private:
    Serial *m_serial;

    std::thread *reading_thread;
    void read_loop();
    int sock;
    struct sockaddr_in _serveraddr;
	struct sockaddr_in _gsaddr;
	struct sockaddr_in _recvfromaddr;
};

#endif

#include <bridge/UDP.h>
#include <bridge/Serial.h>

//----------------------------------------------------------------------------//
int main(int argc, char* argv[])
{
    Serial serial("/dev/ttyACM0", 9600);
    UDP udp("192.168.43.214", 10011, 10010);

    udp.set_serial(&serial);
    serial.set_udp(&udp);

    serial.start();
    udp.start();

    udp.join();
    serial.join();

    return 0;
}
//----------------------------------------------------------------------------//

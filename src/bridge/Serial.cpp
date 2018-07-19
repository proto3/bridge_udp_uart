#include <bridge/Serial.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

using namespace std;

//----------------------------------------------------------------------------//
Serial::Serial(string port , int baudrate)
: reading_thread(NULL), m_udp(NULL)
{
    serial_fd = open(port.c_str(), O_RDWR);

    if(serial_fd == -1)
        throw logic_error(string("cannot open ") + port + ": " + strerror(errno));

    if(!isatty(serial_fd))
        throw logic_error(port + " is not a tty.");

    //TODO why to reset flags on new fd ?
    //aren't they clean ?
    fcntl(serial_fd, F_SETFL, 0);

    // Read file descriptor configuration
    struct termios config;
    if(tcgetattr(serial_fd, &config) < 0)
        throw logic_error(string("Cannot read file descriptor configuration: ") + strerror(errno));

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    #ifdef OLCUC
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 0;

    // Apply baudrate
    int baudrate_val;
    switch (baudrate)
    {
        case 9600:
            baudrate_val = B9600;
            break;
        case 19200:
            baudrate_val = B19200;
            break;
        case 38400:
            baudrate_val = B38400;
            break;
        case 57600:
            baudrate_val = B57600;
            break;
        case 115200:
            baudrate_val = B115200;
            break;
        case 230400:
            baudrate_val = B230400;
            break;
        case 460800:
            baudrate_val = B460800;
            break;
        case 921600:
            baudrate_val = B921600;
            break;
        default:
            throw logic_error("Cannot handle baudrate : " + to_string(baudrate));
            break;
    }

    if(cfsetispeed(&config, baudrate_val) < 0 || cfsetospeed(&config, baudrate_val) < 0)
        throw logic_error("Cannot set baudrate");

    // Finally, apply the configuration
    if(tcsetattr(serial_fd, TCSAFLUSH, &config) < 0)
        throw logic_error("Cannot set file descriptor configuration");

    cout << "Connected to " << port << " " << baudrate << endl;

    //Issue in linux kernel, tcflush is not effective immediately after serial port opening
    usleep(1000000);
    tcflush(serial_fd, TCIOFLUSH);
}
//----------------------------------------------------------------------------//
Serial::~Serial()
{
    close(serial_fd);
}
//----------------------------------------------------------------------------//
bool Serial::is_valid_tty(const char* path)
{
    bool ret = true;
    int fd = open(path, O_RDWR | O_NOCTTY);

    if(fd == -1 || !isatty(fd))
        ret = false;

    close(fd);
    return ret;
}
//----------------------------------------------------------------------------//
void Serial::start()
{
    if(reading_thread == NULL)
    {
        reading_thread = new thread(&Serial::read_loop, this);
    }
}
//----------------------------------------------------------------------------//
void Serial::join()
{
    reading_thread->join();
}
//----------------------------------------------------------------------------//
void Serial::read_loop()
{
    size_t length = 256;
    uint8_t buffer[length];
    while(true)
    {
        ssize_t nb_read = read(serial_fd, buffer, length);
        if(nb_read == -1)
            throw logic_error("Unable to read from serial port.");

        if(m_udp != NULL && nb_read > 0)
            m_udp->send(buffer, nb_read);
    }
}
//----------------------------------------------------------------------------//
bool Serial::send(uint8_t *buffer, uint32_t length)
{
    for(int i = 0; i < length ; i++)
    {
        cout << hex << setw(2) << setfill('0') << (int)buffer[i] << " ";
    }
    cout << dec << endl;
    int bytes_sent = write(serial_fd, buffer, length);
    return length == bytes_sent;
}
//----------------------------------------------------------------------------//
void Serial::set_udp(UDP *udp)
{
    m_udp = udp;
}
//----------------------------------------------------------------------------//

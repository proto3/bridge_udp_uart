#include <bridge/UDP.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

using namespace std;

//----------------------------------------------------------------------------//
UDP::UDP(string target_ip, int target_port, int local_port)
: reading_thread(NULL), m_serial(NULL)
{
    if(target_port < 0 || target_port > 65535)
        throw logic_error(string("Target port outside range."));

    if(local_port < 0 || local_port > 65535)
        throw logic_error(string("Local port outside range."));

    sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		throw system_error(EFAULT, system_category());
	}

	int enable = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const void *)&enable,
		       sizeof(int))
	    == -1) {
		close(sock);
		throw system_error(EFAULT, system_category());
	}

	memset(&_serveraddr, 0, sizeof(struct sockaddr_in));
	_serveraddr.sin_family = AF_INET;
	_serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_serveraddr.sin_port = htons(local_port);

	memset(&_gsaddr, 0, sizeof(struct sockaddr_in));
	_gsaddr.sin_family = AF_INET;
	_gsaddr.sin_port = htons(target_port);

	memset(&_recvfromaddr, 0, sizeof(struct sockaddr_in));

	if (::bind(sock, (struct sockaddr *)&_serveraddr,
		   sizeof(_serveraddr))
	    == -1) {
		close(sock);
		throw system_error(EFAULT, system_category());
	}

	if (fcntl(sock, F_SETFL, FASYNC) == -1) {
		close(sock);
		throw system_error(EFAULT, system_category());
	}

    cout << "to " << target_ip << ":" << target_port << endl;

    reading_thread = new thread(&UDP::read_loop, this);
}
//----------------------------------------------------------------------------//
UDP::~UDP()
{
    close(sock);
}
//----------------------------------------------------------------------------//
bool UDP::is_valid_ip(const char* ip)
{
    char* dst[INET_ADDRSTRLEN];
    return inet_pton(AF_INET, ip, dst) == 1;
}
//----------------------------------------------------------------------------//
void UDP::start()
{
    if(reading_thread == NULL)
    {
        reading_thread = new thread(&UDP::read_loop, this);
    }
}
//----------------------------------------------------------------------------//
void UDP::join()
{
    reading_thread->join();
}
//----------------------------------------------------------------------------//
// Thread loop to move data from RX buffer to RAM as fast as possible
// so as to avoid buffer overflows if RX buffer is small.
void UDP::read_loop()
{
    size_t length = 256;
    uint8_t buffer[length];
    while(true)
    {
        socklen_t fromlen = sizeof(struct sockaddr);
        ssize_t nb_read =
            recvfrom(sock, (void *)buffer, length, 0,
                 (struct sockaddr *)&_recvfromaddr, &fromlen);
        _gsaddr.sin_addr.s_addr = _recvfromaddr.sin_addr.s_addr;

        if(nb_read == -1)
            throw logic_error("Unable to read from UDP socket.");

        if(m_serial != NULL && nb_read > 0)
        {
            m_serial->send(buffer, nb_read);
        }
    }
}
//----------------------------------------------------------------------------//
bool UDP::send(uint8_t *buffer, uint32_t length)
{
    for(int i = 0; i < length ; i++)
    {
        cout << hex << setw(2) << setfill('0') << (int)buffer[i] << " ";
    }
    cout << dec << endl;
    ssize_t bytes_sent = sendto(sock, buffer, length, 0, (struct sockaddr*)&_gsaddr, sizeof(struct sockaddr_in));
    return length == bytes_sent;
}
//----------------------------------------------------------------------------//
void UDP::set_serial(Serial *serial)
{
    m_serial = serial;
}
//----------------------------------------------------------------------------//

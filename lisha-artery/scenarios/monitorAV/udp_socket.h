#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>

class UDP_Socket {
private:
	static constexpr const char * GRP_ADDRESS = "224.1.1.1";
	static const unsigned int PORT  = 5000;

public:
	static const unsigned int MTU   = 65536; // 64KB

public:
	UDP_Socket(unsigned int port_offset = 0, bool use_receiver = true) {

		_group_socket.sin_family = AF_INET;
		_group_socket.sin_addr.s_addr = inet_addr(GRP_ADDRESS);
		_group_socket.sin_port = htons(PORT + port_offset);
		_socket[0] = -1; // send
		_socket[1] = -1; // receive
        _size = 0;
        for (unsigned int i = 0; i < MTU; i++)
        {
            _data[i] = 0;
        }
        

		set_sockets();
	}

	virtual ~UDP_Socket() { }

	const unsigned char * data() { return _data; }
	unsigned int size() { return _size; }

	const unsigned char * receive() {
		int result = 0;
		unsigned int old_size = _size;

		while (true)
		{
			result = read(_socket[1], _data, MTU);
			if (result > 0) {
				_size = result;
				if (_size < old_size)
					for(unsigned int i = _size; i < old_size; i++)
						_data[i] = 0;
				break;
			}
		}
		return _data;
	}

	int send(const unsigned char * data, unsigned int size) {
		return sendto(_socket[0], data, size, 0,(sockaddr*)&_group_socket, sizeof(_group_socket));
	}

private:
	int set_sockets() {
		_socket[0] = socket(AF_INET, SOCK_DGRAM, 0);
		_socket[1] = socket(AF_INET, SOCK_DGRAM, 0);
		receive_binding();
		
		return _socket[0] + _socket[1];
	}

	int receive_binding() {
		int reuse = 1;
		int result = setsockopt(_socket[1], SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse));
		if (result < 0) {
			return -1;
		}

		result = bind(_socket[1], (sockaddr*)&_group_socket, sizeof(_group_socket));
		if (result < 0) {
			return -1;
		}
		return 0;
	}

private:
	sockaddr_in _group_socket;
	int _socket[2];
	unsigned int _size;
	unsigned char _data[MTU];
};
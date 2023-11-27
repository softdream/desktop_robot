#ifndef __DATA_TRANSPORT_H
#define __DATA_TRANSPORT_H

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string.h>
#include <vector>

#include <iostream>

namespace transport
{

class UdpServer
{
public:
	UdpServer() = delete;

	UdpServer( const int port )
	{
		initUdpServer( port );
		setNonBlock();
	}

	~UdpServer()
	{
	
	}

	bool initUdpServer( const int port )
	{
		sock_fd_ = ::socket( AF_INET, SOCK_DGRAM, 0 );
		if ( sock_fd_ < 0 ) {
			std::cerr<<"Cannot Create the socket : "<<sock_fd_<<std::endl;
			
			return false;
		}

		server_addr_.sin_family = AF_INET;
		server_addr_.sin_addr.s_addr = htonl( INADDR_ANY );
		server_addr_.sin_port = htons( port );

		if ( ::bind( sock_fd_, ( struct sockaddr* )&server_addr_, sizeof( server_addr_ ) ) < 0 ) {
			std::cerr<<"Failed to bind the socket server address !"<<std::endl;
			return false;
		}

		std::cout<<"Init Udp Socket : "<<sock_fd_<<std::endl;

		return true;
	}

	const int getSocketFd() const 
	{
		return sock_fd_;
	}

	const int read( char* recv_buffer, const int len )
	{
		memset( recv_buffer, 0, len );

		int ret = ::recvfrom( sock_fd_, recv_buffer, len, 0, ( struct sockaddr* )&client_addr_, &client_sock_len_ );
		if ( ret <= 0 ) {
			std::cerr<<"recv failed : "<<ret<<std::endl;
			return ret;
		}

		return ret;
	}

	const int write( const char* data, const int len )
	{
		return ::sendto( sock_fd_, data, len, 0, ( struct sockaddr* )&client_addr_, client_sock_len_ );
	}

	const int write( const char* data, const int len, const std::string ip, const int port )
	{
		struct sockaddr_in client_addr;

                client_addr.sin_family = AF_INET;
                client_addr.sin_addr.s_addr = inet_addr( ip.c_str() );
                client_addr.sin_port = htons( port );

		return ::sendto( sock_fd_, data, len, 0, ( struct sockaddr* )&client_addr, sizeof( client_addr ) );
	}

	bool setNonBlock()
	{
		int flag = ::fcntl( sock_fd_, F_GETFL, 0 );
		if ( flag < 0 ) {
			std::cerr<<"fcntl F_GETFL failed !"<<std::endl;
			return false;
		}

		if ( ::fcntl( sock_fd_, F_SETFL, flag | O_NONBLOCK ) < 0 ) {
			std::cerr<<"fcntl F_SETFL failed !"<<std::endl;
			return false;
		}

		return true;
	}

protected:
	// server info
	int sock_fd_ = -1;
	struct sockaddr_in server_addr_;

	// remote client info
	struct sockaddr_in client_addr_;
	socklen_t client_sock_len_;
};

class Receiver : public UdpServer
{
public:
	Receiver() = delete;

	Receiver( const int port ) : UdpServer( port )
	{
		
	}	

	~Receiver()
	{
	
	}


        template<typename T>
        int send( const T& data )
        {
                return this->write( (char*)&data, sizeof( data ) );
        }

        template<typename T>
        int send( const std::vector<T>& vec )
        {
                return this->write( (char*)vec.data(), vec.size() * sizeof( T ) );
        }

	template<typename T>
	int send( const T& data, const std::string ip, const int port)
	{
		return this->write( (char*)&data, sizeof( data ), ip, port );
	}

	template<typename T>
	int send( const T& data, const char header, const std::string ip, const int port )
	{
		char buffer[sizeof( data ) + 1] = {0};
		buffer[0] = header;
		memcpy( &buffer[1], &data, sizeof( data ) );
		return this->write( buffer, sizeof( data ) + 1, ip, port );
	}
};

class UdpClient
{
public:
	UdpClient()
	{
		initUdpClient();
	}

	UdpClient( const int port, const std::string& ip )
	{
		initUdpClient( port, ip );
	}

	~UdpClient()
	{
	
	}

	bool initUdpClient()
	{
		sock_fd_ = ::socket( AF_INET, SOCK_DGRAM, 0 );
		if ( sock_fd_ < 0 ) {
			std::cerr<<"socket Udp Client Failed !"<<std::endl;
			return false;
		}

		std::cout<<"Initialized the Udp Client !"<<std::endl;
		return true;
	}

	bool initUdpClient( const int port, const std::string& ip )
	{
		dst_addr_.sin_family = AF_INET;
                dst_addr_.sin_addr.s_addr = inet_addr( ip.c_str() );
                dst_addr_.sin_port = htons( port );

		return initUdpClient();
	}

	const int write( const char* data, const int len )
	{
		return ::sendto( sock_fd_, data, len, 0, (struct sockaddr*)&dst_addr_, sizeof( dst_addr_ ) );
	}

	const int write( const char* data, const int len, const int port, const std::string& ip )
	{
		dst_addr_.sin_family = AF_INET;
		dst_addr_.sin_addr.s_addr = inet_addr( ip.c_str() );
		dst_addr_.sin_port = htons( port );

		return ::sendto( sock_fd_, data, len, 0, (struct sockaddr*)&dst_addr_, sizeof( dst_addr_ ) );
	}

private:
	int sock_fd_ = -1;
	struct sockaddr_in dst_addr_;
};

}

#endif

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

#include <vector>

namespace transport
{

template<unsigned BufferSize>
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

	const int read()
	{
		memset( recv_buffer_, 0, sizeof( recv_buffer_ ) );

		int ret = ::recvfrom( sock_fd_, recv_buffer_, sizeof( recv_buffer_ ), 0, ( struct sockaddr* )&client_addr_, &client_sock_len_ );
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

	// receive buffer
	char recv_buffer_[BufferSize];
};

template<unsigned BufferSize = 50>
class Receiver : public UdpServer<BufferSize>
{
public:
	Receiver() = delete;

	Receiver( const int port ) : UdpServer<BufferSize>( port )
	{
		
	}	

	~Receiver()
	{
	
	}

	template<typename T>
	bool receive( T& data )
	{
		if ( this->read() ) {
			memcpy( &data, this->recv_buffer_, sizeof( data ) );
			
			return true;
		}
		else {
			return false;
		}
	}

	bool receive( std::string& data )
        {
                if ( this->read() ) {
                        data = this->recv_buffer_;
                        return true;
                }
                else {
                        return false;
                }
        }

	template<typename T>
	void send( const T& data )
	{
		return this->write( (char*)&data, sizeof( data ) );
	}

	template<typename T>
	void send( const std::vector<T>& vec ) 
	{
		return this->write( (char*)vec.data(), vec.size() * sizeof( T ) );
	}
};

}

#endif

#ifndef EVT_TCP_PUBLISHER_H
#define EVT_TCP_PUBLISHER_H

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <string>

#include "event_framework.h"

using boost::asio::ip::tcp;

// This filter marks all filtered events as valid again
// Arguments: none
class evt_tcp_publisher: public DataConsumer {
public:
	static const int HEADER_SIZE = 4;
	evt_tcp_publisher(std::string ip_add_in, int port_in);

	bool processData(packet_info_type* packet_info, event* dataIN,
			bool exit_thread);

private:
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket * socket;
	std::vector<uint8_t> data_buffer;
};

#endif

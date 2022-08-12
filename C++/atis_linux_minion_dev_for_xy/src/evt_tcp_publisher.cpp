#include "../inc/evt_tcp_publisher.h"
#include <boost/asio.hpp>
#include <boost/cstdint.hpp>
//#include <boost/asio/basic_stream_socket.hpp>
//#include <boost/asio/buffer.hpp>
//#include <boost/asio/impl/connect.hpp>
//#include <boost/asio/ip/basic_resolver.hpp>
//#include <boost/asio/ip/tcp.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <exception>
#include <iostream>
#include <system_error>

#include "protobuf/events.pb.h"

using namespace std;
using boost::asio::ip::tcp;

evt_tcp_publisher::evt_tcp_publisher(string ip_add_in, int port_in) {
	try {
		socket = new boost::asio::ip::tcp::socket(io_service);
		tcp::resolver resolver(io_service);
		tcp::resolver::query query1(ip_add_in,
				boost::lexical_cast<string>(port_in));
		tcp::resolver::iterator endpoint = resolver.resolve(query1);
		connect(*socket, endpoint);
	} catch (std::system_error &e) {
		std::cerr << "EVT_TCP_PUBLISHER Exception: " << e.what() << std::endl;
	}
}

bool evt_tcp_publisher::processData(packet_info_type* packet_info,
		event* dataIN, bool exit_thread) {
//	std::cout << "Processing..." << std::endl;
	if (packet_info->packet_type == packet_type::TD_packet
			|| packet_info->packet_type == packet_type::TD_EM_mixed_packet) {
		epf::Events events = epf::Events();

//		std::cout << "\t " << packet_info->num_events << " events" << std::endl;
		for (int i = 0; i < packet_info->num_events; i++) {
			auto data = dataIN[i];
			if (data.type == evt_type::TD
					|| data.type == evt_type::TD_filtered) {
				epf::Event * evt = events.add_evt();
				evt->set_x(data.x);
				evt->set_y(data.y);
				evt->set_p(data.subtype);

//			if (data.x ==0 || data.y == 0 || data.subtype == 0) {
//				std::cout << i << ": " << data.x << ", " << data.y << ", "
//						<< (int) data.type << ", " << (int) data.subtype << std::endl;
//			}
			}
		}

		if (events.evt_size() > 0) {
//			std::cout << "\tPublishing..." << std::endl;
			int msg_size = events.ByteSize();

			data_buffer.resize(msg_size + HEADER_SIZE);

			//encode 4 bytes header to denote size of message body
			data_buffer[0] =
					static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
			data_buffer[1] =
					static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
			data_buffer[2] =
					static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
			data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

			//encode message body
			events.SerializeToArray(&data_buffer[HEADER_SIZE], msg_size);
			socket->send(
					boost::asio::buffer(data_buffer, msg_size + HEADER_SIZE));

			{
//			epf::Events events2;
//			events2.ParseFromArray(&data_buffer[HEADER_SIZE], msg_size);
//			std::cout << "parsed" << std::endl;
			}
		}
	}

	return exit_thread;
}


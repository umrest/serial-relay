#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>

#include <boost/array.hpp>

#include <comm/CommunicationDefinitions.h>
#include <comm/Identifier.h>

#include <iostream>

#include <chrono>
#include <thread>

class SerialRelay
{
public:
    // Hero
    std::string serial_device;
    std::string ip;
    int baud;
    int port;
    comm::CommunicationDefinitions::IDENTIFIER identifier_id;

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;

    boost::asio::serial_port serial;

    unsigned char serial_buffer[1024]; // buffer from serial
    unsigned char socket_buffer[1024]; // buffer from socket

    bool socket_connected = false;
    bool serial_connected = false;

    SerialRelay(std::string _ip, int _port, std::string _serial_device, int _baud, comm::CommunicationDefinitions::IDENTIFIER _identifier_id) : socket(io_service), serial(io_service), serial_device(_serial_device), ip(_ip), port(_port), baud(_baud), identifier_id(_identifier_id)
    {
        socket_reconnect();
        serial_reconnect();
    }

    void handle_serial_receive(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        if (!ec)
        {
            boost::system::error_code ec2;
            socket.write_some(boost::asio::buffer(serial_buffer, bytes_transferred), ec2);
            if (ec2)
            {
                socket_connected = false;
            }
            serial_read();
        }
        else
        {
            std::cout << "Serial: Disconnected" << std::endl;
            serial.close();
            serial_reconnect();
        }
    }

    void handle_socket_receive(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        if (!ec)
        {
            boost::system::error_code ec2;
            serial.write_some(boost::asio::buffer(socket_buffer, bytes_transferred), ec2);
            if (ec2)
            {
                serial_connected = false;
            }
            socket_read();
        }
        else
        {
            std::cout << "Socket: Disconnected" << std::endl;
            socket.close();
            socket_reconnect();
        }
    }

    void socket_read()
    {
        socket.async_read_some(boost::asio::buffer(socket_buffer),
                               boost::bind(&SerialRelay::handle_socket_receive,
                                           this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
    }

    void serial_read()
    {
        serial.async_read_some(boost::asio::buffer(serial_buffer),
                               boost::bind(&SerialRelay::handle_serial_receive,
                                           this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
    }

    void serial_reconnect()
    {
        boost::system::error_code ec;

        serial.open(serial_device, ec);

        if (!ec)
        {
            boost::asio::serial_port_base::baud_rate baud_rate(baud);
            serial.set_option(baud_rate);
            serial_connected = true;
            std::cout << "Serial: Reconnect Succeeded" << std::endl;
            serial_read();
        }
        else
        {
            serial_connected = false;
            std::cout << "Serial: Reconnect Failed" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            serial_reconnect();
        }
    }

    void socket_reconnect()
    {
        boost::system::error_code ec;

        socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip), port), ec);

        if (!ec)
        {
            socket_connected = true;
            std::cout << "Socket: Reconnect Succeeded" << std::endl;

            comm::Identifier identifier;
		    identifier.set_identifier((uint8_t)identifier_id);

            auto data = identifier.Serialize();

            std::vector<uint8_t> type;
            type.push_back((uint8_t)comm::CommunicationDefinitions::TYPE::IDENTIFIER);

            boost::system::error_code ec;

            boost::array<boost::asio::const_buffer, 3> d = {
                boost::asio::buffer(comm::CommunicationDefinitions::key, 3),
                boost::asio::buffer(type, type.size()),
                boost::asio::buffer(data, data.size()) };

            int bytesTransferred = socket.write_some(d, ec);

            socket_read();
        }
        else
        {
            socket_connected = false;
            std::cout << "Socket: Reconnect Failed" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            socket_reconnect();
        }
    }

    void run()
    {
        io_service.run();
    }
};

int main(int argc, char *argv[])
{
    std::string identifier(argv[2]);
    comm::CommunicationDefinitions::IDENTIFIER identifier_id;
    if(identifier == "hardware"){
        identifier_id = comm::CommunicationDefinitions::IDENTIFIER::HARDWARE;
    }
    if(identifier == "hero"){
        identifier_id = comm::CommunicationDefinitions::IDENTIFIER::TCPSERIAL;
    }
    SerialRelay sr("127.0.0.1", 8091, std::string(argv[1]), 115200, identifier_id);
    sr.run();
}
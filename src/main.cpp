#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <iostream>

#include <chrono>
#include <thread>



class SerialRelay {
    public:
    int baud = 115200;
    std::string port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75438313733351D06252-if00";

    boost::asio::io_service    io_service;
    boost::asio::ip::tcp::socket socket;

    boost::asio::serial_port  serial;

    unsigned char serial_buffer[1024]; // buffer from serial
    unsigned char socket_buffer[1024]; // buffer from socket

    bool socket_connected = false;
    bool serial_connected = false;
    
    SerialRelay(std::string ip, int port, std::string serial_device, int baud) : socket(io_service), serial(io_service){
       socket_reconnect();
       serial_reconnect();

       serial_read();
       socket_read();
    }

   

    void handle_serial_receive(const boost::system::error_code& ec, std::size_t bytes_transferred){
        if(!ec){
            boost::system::error_code ec2;
            socket.write_some(boost::asio::buffer(serial_buffer, bytes_transferred), ec2);
            if(ec2){
                socket_connected = false;
            }
            serial_read();
        }
        else{
            std::cout << "Serial: Disconnected" << std::endl;
            serial.close();
            serial_reconnect();
            serial_read();
        }
    }

  

    void handle_socket_receive(const boost::system::error_code& ec, std::size_t bytes_transferred){
        if(!ec){
            boost::system::error_code ec2;
            serial.write_some(boost::asio::buffer(socket_buffer, bytes_transferred), ec2);
            if(ec2){
                serial_connected = false;
            }
            socket_read();
        }
        else{
            std::cout << "Socket: Disconnected" << std::endl;
            socket.close();
            socket_reconnect();
            socket_read();
        }
    }

    void socket_read(){
        socket.async_read_some(boost::asio::buffer(socket_buffer),
            boost::bind(&SerialRelay::handle_socket_receive,
                this, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }

     void serial_read(){
        serial.async_read_some(boost::asio::buffer(serial_buffer),
            boost::bind(&SerialRelay::handle_serial_receive,
                this, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }

    void serial_reconnect(){
        boost::system::error_code ec;

        serial.open(port, ec);

        if(!ec){
            boost::asio::serial_port_base::baud_rate baud_rate(baud);   
            serial.set_option(baud_rate);
            serial_connected = true;
            std::cout << "Serial: Reconnect Succeeded" << std::endl;
        }
        else{
            serial_connected = false;
            std::cout << "Serial: Reconnect Failed" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void socket_reconnect(){
        boost::system::error_code ec;

        socket.connect(boost::asio::ip::tcp::endpoint( boost::asio::ip::address::from_string("127.0.0.1"), 8091 ), ec);

        if(!ec){
            socket_connected = true;
            std::cout << "Socket: Reconnect Succeeded" << std::endl;

            char identifier[128];
            identifier[0] = 250;
            identifier[1] = 3;
            socket.write_some(boost::asio::buffer(identifier, 128));
        }
        else{
            socket_connected = false;
            std::cout << "Socket: Reconnect Failed" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void run(){
        io_service.run();
    }


};

int main(){
    SerialRelay sr("127.0.0.1", 8091, "/dev/serial/by-id/...", 115200);
    sr.run();
}
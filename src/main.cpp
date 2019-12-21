#include <boost/asio/serial_port.hpp>

int baud = 115200;
int port = "/dev/serial/by-id/...";

int main(){
    boost::asio::io_service    m_ioService;  //order is important so that m_ioService gets constructed first
   boost::asio::serial_port   m_port;

   boost::system::error_code ec;  // choice: without ec Boost.Asio may throw

   m_port.open(port, ec);

   if (!ec)
   {
       boost::asio::serial_port_base::baud_rate baud_rate(baud);   
       m_port.set_option(baud_rate);

       
   }
}
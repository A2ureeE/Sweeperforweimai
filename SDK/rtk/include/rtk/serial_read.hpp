#ifndef __UART_READ_H__
#define __UART_READ_H__

#include <boost/asio.hpp>
#include <iostream>

using namespace std;
using namespace boost::asio;

class Boost_serial
{
   public:
    Boost_serial();
    ~Boost_serial();
    void init(const string port_name);
    int serial_read(char buf[], int size);

   private:
    io_service service;
    serial_port* sp;
};

#endif
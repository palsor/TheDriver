#include "Communication.h"

//
// constructor
// 
Communication::Communication() : port(SOFT_SERIAL_RX, SOFT_SERIAL_TX) {}

//
// init
//
void Communication::init() {
  port.begin(SOFT_SERIAL_RATE);
}

//
// print - send data over serial
//
void Communication::print() {
  
}


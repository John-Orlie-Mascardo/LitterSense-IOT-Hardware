#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <deque>
#include <sstream>
using String = std::string;
#define OUTPUT 1
#define HIGH 1
#define SERIAL_8N1 0
static uint32_t clockMs=0;
inline uint32_t millis(){return clockMs;}
inline void delay(uint32_t n){clockMs+=n;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
struct HardwareSerial {
 std::ostringstream output;
 std::deque<uint8_t> rx;
 HardwareSerial(int=0){}
 void begin(int,int=0,int=0,int=0){}
 int available(){return rx.size();}
 int availableForWrite(){return 256;}
 int read(){auto b=rx.front();rx.pop_front();return b;}
 void write(const uint8_t*,size_t){}
 template<class T> void print(T value){output << value;}
 template<class T> void println(T value){output << value << '\n';}
 void println(){output << '\n';}
};
static HardwareSerial Serial;

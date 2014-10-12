#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "HardwareSerial.h"
#include "Arduino.h"

class Bluetooth
{
public:
    Bluetooth();
    ~Bluetooth();
    static boolean setBluetoothBaud(HardwareSerial& setSerial, const long baud);
    static boolean handshake(HardwareSerial& setSerial, const long baud);
};
#endif
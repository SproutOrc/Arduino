#include "Bluetooth.h"
#include "Arduino.h"

const long baudArray[] = {
    1200, 2400, 4800, 
    9600, 19200, 38400, 
    57600, 115200
};

// 1---------1200
// 2---------2400
// 3---------4800
// 4---------9600
// 5---------19200
// 6---------38400
// 7---------57600
// 8---------115200

const char AT[] = {
    'A', 'T'
};

char setBaud[8] = {
    'A', 'T', '+', 'B', 'A', 'U', 'D'
};

boolean Bluetooth::setBluetoothBaud(HardwareSerial& setSerial, const long baud) {

    if (handshake(setSerial, baud)) return true;

    byte i;
    for (i = 0; i < 8; ++i) {
        if (baud == baudArray[i]) {
            setBaud[7] = i + 1 + '0';
            break;
        }
    }

    if (i == 8) return false;
    
    for (i = 0; i < 8; ++i) {
        if (handshake(setSerial, baudArray[i])) {
            delay(1000);
            setSerial.write(setBaud, 8);
            delay(1000);
            setSerial.begin(baud);
            delay(5);
            return true;
        }
    }

    if (i == 8) return false;
}

boolean Bluetooth::handshake(HardwareSerial& setSerial, const long baud) {
    setSerial.begin(baud);
    delay(1);
    setSerial.write(AT, 2);
    delay(1000);
    if (setSerial.available() >= 2) {
        char receive[2];
        setSerial.readBytes(receive, 2);
        Serial.flush();
        if (receive[0] == 'O' && receive[1] == 'K') {
            return true;
        }
    }
    return false;
}

// if (setSerial.available() >= 2) {
//             char receive[2];
//             setSerial.readBytes(receive, 2);
//             Serial.flush();
//             if (receive[0] == 'O' && receive[1] == 'K') {
//                 delay(1000);
//                 setSerial.write(setBaud, 8);
//                 delay(1000);
//                 setSerial.begin(baud);
//                 setSerial.println("set successful!");
//                 delay(5);
//                 return true;
//             }
//         }
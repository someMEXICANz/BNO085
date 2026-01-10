#include "BNO085.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>

#include <open3d/Open3D.h>

int main() {
    // Use Pin 7 (gpiochip0 line 105)
    BNO085 sensor("/dev/i2c-7", 0x4A, BNO085::ResetPin::PIN_7);

    sensor.initialize();
    sensor.startService();



    while (true) {
        //std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
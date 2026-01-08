#include "BNO085.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>

int main() {
    // Use Pin 7 (gpiochip0 line 105)
    BNO085 sensor("/dev/i2c-7", 0x4A, BNO085::ResetPin::PIN_7);
    
    if (!sensor.initialize()) {
        std::cerr << "Failed to initialize: " << sensor.getLastError() << std::endl;
        return 1;
    }

    
    
    if (sensor.hardwareReset()) {
        std::cout << "✓ Hardware reset is working!" << std::endl;
    }

    sensor.startService();

    while(true)
    {

    }
    
    //std::cout << sensor.getProductInfo() << std::endl;
    
    return 0;
}
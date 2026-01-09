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

    sensor.startService();


    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "Retrieving Sensor info in" << std::endl;
    for(int i = 0; i < 5; i++) 
    {
        std::cout << 5 - i << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << sensor.getProductInfo() << std::endl;


    

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "Resetting sensor via hardware pin in " << std::endl;
    for(int i = 0; i < 5; i++) 
    {
        std::cout << 5 - i << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    sensor.hardwareReset();
    std::cout << "Reset command issued." << std::endl;
    
   
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // while (true) {
    //     //std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    
    return 0;
}



#include "BNO085.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <memory>

#include "Visualization.h"

// =============================================================================
// CONFIGURATION - EDIT THESE VALUES
// =============================================================================

const std::string MODEL_PATH = "models/bun_zipper.ply";      // Path to your 3D model
const std::string I2C_BUS = "/dev/i2c-7";               // I2C bus
const uint8_t I2C_ADDRESS = 0x4A;                       // I2C address
const BNO085::ResetPin RESET_PIN = BNO085::ResetPin::PIN_7;  // Reset pin


// =============================================================================
// MAIN
// =============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "BNO085 IMU 3D Visualization" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Model: " << MODEL_PATH << std::endl;
    std::cout << "I2C: " << I2C_BUS << " @ 0x" << std::hex 
              << static_cast<int>(I2C_ADDRESS) << std::dec << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Initialize sensor
    std::cout << "Initializing BNO085..." << std::endl;
    BNO085 sensor(I2C_BUS, I2C_ADDRESS, RESET_PIN);
    
    if (!sensor.initialize()) {
        std::cerr << "Failed to initialize: " << sensor.getLastError() << std::endl;
        return 1;
    }
    
    startVisualization(sensor, MODEL_PATH);
    sensor.stopService();
    sensor.shutdown();
    
    std::cout << "Done!" << std::endl;
    return 0;
}



































// #include "BNO085.h"
// #include <iostream>
// #include <iomanip>
// #include <signal.h>
// #include <thread>
// #include <chrono>

// #include <open3d/Open3D.h>

// int main() {
//     // Use Pin 7 (gpiochip0 line 105)
//     BNO085 sensor("/dev/i2c-7", 0x4A, BNO085::ResetPin::PIN_7);

//     sensor.initialize();
//     sensor.startService();



//     while (true) {
//         //std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
    
//     return 0;
// }


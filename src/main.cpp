


// // #include "BNO085.h"
// // #include <iostream>
// // #include <iomanip>
// // #include <signal.h>
// // #include <thread>
// // #include <chrono>
// // #include <atomic>
// // #include <mutex>
// // #include <memory>

// // #include "Visualization.h"

// // // =============================================================================
// // // CONFIGURATION - EDIT THESE VALUES
// // // =============================================================================

// // const std::string MODEL_PATH = "models/bun_zipper.ply";      // Path to your 3D model
// // const std::string I2C_BUS = "/dev/i2c-7";               // I2C bus
// // const uint8_t I2C_ADDRESS = 0x4A;                       // I2C address
// // const BNO085::ResetPin RESET_PIN = BNO085::ResetPin::PIN_7;  // Reset pin


// // // =============================================================================
// // // MAIN
// // // =============================================================================

// // int main() {
// //     std::cout << "========================================" << std::endl;
// //     std::cout << "BNO085 IMU 3D Visualization" << std::endl;
// //     std::cout << "========================================" << std::endl;
// //     std::cout << "Model: " << MODEL_PATH << std::endl;
// //     std::cout << "I2C: " << I2C_BUS << " @ 0x" << std::hex 
// //               << static_cast<int>(I2C_ADDRESS) << std::dec << std::endl;
// //     std::cout << "========================================\n" << std::endl;
    
// //     signal(SIGINT, signalHandler);
// //     signal(SIGTERM, signalHandler);
    
// //     // Initialize sensor
// //     std::cout << "Initializing BNO085..." << std::endl;
// //     BNO085 sensor(I2C_BUS, I2C_ADDRESS, RESET_PIN);
    
// //     if (!sensor.initialize()) {
// //         std::cerr << "Failed to initialize: " << sensor.getLastError() << std::endl;
// //         return 1;
// //     }
    
// //     startVisualization(sensor, MODEL_PATH);
// //     sensor.stopService();
// //     sensor.shutdown();
    
// //     std::cout << "Done!" << std::endl;
// //     return 0;
// // }


















// // main.cpp - Enhanced with Interactive GUI
// #include "BNO085.h"
// #include "GUI.h"
// #include <iostream>
// #include <thread>
// #include <atomic>
// #include <signal.h>

// std::atomic<bool> running{true};

// void signalHandler(int signal) {
//     std::cout << "\nShutting down..." << std::endl;
//     running = false;
// }

// void sensorUpdateLoop(IMUVisualizer* visualizer, BNO085& sensor) {
//     while (running && visualizer->IsRunning()) {
//         BNO085::OrientationData orient_data;
//         BNO085::IMUData imu_data;
        
//         // Try to get data from any active sensor
//         if (sensor.getOrientationData(orient_data) && 
//             sensor.getIMUData(imu_data)) {
            
//             visualizer->UpdateSensorData(orient_data.rotation, 
//                                         imu_data.acceleration);
//         }
        
//         std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
//     }
//     running = false;
// }

// int main(int argc, char* argv[]) {
//     // ==========================================================================
//     // CONFIGURATION - EDIT THESE VALUES
//     // ==========================================================================
//     const std::string MODEL_PATH = "models/bun_zipper.ply";  // Path to your 3D model
//     const std::string I2C_BUS = "/dev/i2c-7";                 // I2C bus
//     const uint8_t I2C_ADDRESS = 0x4A;                         // I2C address
//     const BNO085::ResetPin RESET_PIN = BNO085::ResetPin::PIN_7;  // Reset pin
    
//     // ==========================================================================
//     // SETUP
//     // ==========================================================================
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);
    
//     std::cout << "========================================" << std::endl;
//     std::cout << "BNO085 IMU Visualizer with Controls" << std::endl;
//     std::cout << "========================================" << std::endl;
//     std::cout << "Model: " << MODEL_PATH << std::endl;
//     std::cout << "I2C: " << I2C_BUS << " @ 0x" << std::hex 
//               << static_cast<int>(I2C_ADDRESS) << std::dec << std::endl;
//     std::cout << "========================================\n" << std::endl;
    
//     // Initialize sensor
//     std::cout << "Initializing BNO085..." << std::endl;
//     BNO085 sensor(I2C_BUS, I2C_ADDRESS, RESET_PIN);
    
//     if (!sensor.initialize()) {
//         std::cerr << "Failed to initialize sensor: " << sensor.getLastError() << std::endl;
//         return 1;
//     }
    
//     // Start with default sensors (rotation vector and basic IMU)
//     std::cout << "Enabling default sensors..." << std::endl;
//     sensor.enableOrientation();
//     sensor.enableBasicIMU();
//     sensor.startService();
    
//     // Wait for initial data
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     std::cout << "Sensor initialized successfully" << std::endl;
    
//     // ==========================================================================
//     // GUI SETUP
//     // ==========================================================================
    
//     // Initialize GUI application
//     std::cout << "Initializing Open3D GUI..." << std::endl;
//     open3d::visualization::gui::Application::GetInstance().Initialize("/usr/local/share/Open3D/resources");
    
//     // Create visualizer (pass sensor reference so GUI can control it)
//     auto visualizer = std::make_shared<IMUVisualizer>(MODEL_PATH, sensor);
//     visualizer->Run();
    
//     // Start sensor update thread
//     std::cout << "Starting sensor update thread..." << std::endl;
//     std::thread update_thread(sensorUpdateLoop, visualizer.get(), std::ref(sensor));
    
//     // ==========================================================================
//     // RUN GUI (BLOCKING)
//     // ==========================================================================
//     std::cout << "Starting GUI (use controls to adjust sensors)..." << std::endl;
//     std::cout << "\nControls:" << std::endl;
//     std::cout << "  - Select sensor type from dropdown" << std::endl;
//     std::cout << "  - Adjust sampling rate" << std::endl;
//     std::cout << "  - Enable/disable sensors with checkbox" << std::endl;
//     std::cout << "  - Monitor calibration status" << std::endl;
//     std::cout << "  - Start calibration and save when complete\n" << std::endl;
    
//     open3d::visualization::gui::Application::GetInstance().Run();
    
//     // ==========================================================================
//     // CLEANUP
//     // ==========================================================================
//     std::cout << "Cleaning up..." << std::endl;
//     running = false;
//     update_thread.join();
    
//     sensor.stopService();
//     sensor.shutdown();
    
//     std::cout << "Done!" << std::endl;
//     return 0;
// }



// // // main.cpp - Revised
// // #include "BNO085.h"
// // #include "GUI.h"
// // #include <iostream>
// // #include <thread>
// // #include <atomic>
// // #include <signal.h>

// // std::atomic<bool> running{true};

// // void signalHandler(int signal) {
// //     std::cout << "\nShutting down..." << std::endl;
// //     running = false;
// // }

// // void sensorUpdateLoop(IMUVisualizer* visualizer, BNO085& sensor) {
// //     while (running && visualizer->IsRunning()) {
// //         BNO085::OrientationData orient_data;
// //         BNO085::IMUData imu_data;
        
// //         if (sensor.getOrientationData(orient_data) && 
// //             sensor.getIMUData(imu_data)) {
            
// //             visualizer->UpdateSensorData(orient_data.rotation, 
// //                                         imu_data.acceleration);
// //         }
        
// //         std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
// //     }
// //     running = false;
// // }

// // int main(int argc, char* argv[]) {
// //     const std::string MODEL_PATH = "models/bun_zipper.ply";
// //     const std::string I2C_BUS = "/dev/i2c-7";
// //     const uint8_t I2C_ADDRESS = 0x4A;
// //     const BNO085::ResetPin RESET_PIN = BNO085::ResetPin::PIN_7;
    
// //     signal(SIGINT, signalHandler);
// //     signal(SIGTERM, signalHandler);
    
// //     std::cout << "========================================" << std::endl;
// //     std::cout << "BNO085 IMU Visualizer" << std::endl;
// //     std::cout << "========================================" << std::endl;
    
// //     // Initialize sensor
// //     BNO085 sensor(I2C_BUS, I2C_ADDRESS, RESET_PIN);
// //     if (!sensor.initialize()) {
// //         std::cerr << "Failed to initialize sensor" << std::endl;
// //         return 1;
// //     }
    
// //     sensor.enableOrientation();
// //     sensor.enableBasicIMU();
// //     sensor.startService();
    
// //     // Wait for initial data
// //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
// //     std::cout << "Sensor initialized" << std::endl;
    
// //     // Initialize GUI application
// //     open3d::visualization::gui::Application::GetInstance().Initialize("/usr/local/share/Open3D/resources");
    
// //     // Create visualizer
// //     auto visualizer = std::make_shared<IMUVisualizer>(MODEL_PATH);
// //     visualizer->Run();
    
// //     // Start sensor update thread
// //     std::thread update_thread(sensorUpdateLoop, visualizer.get(), std::ref(sensor));
    
// //     // Run GUI (blocking)
// //     std::cout << "Starting GUI..." << std::endl;
// //     open3d::visualization::gui::Application::GetInstance().Run();
    
// //     // Cleanup
// //     std::cout << "Cleaning up..." << std::endl;
// //     running = false;
// //     update_thread.join();
// //     sensor.shutdown();
    
// //     std::cout << "Done!" << std::endl;
// //     return 0;
// // }












#include "BNO085.h"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <thread>
#include <chrono>

#include <open3d/Open3D.h>

int main() {
    // Use Pin 7 (gpiochip0 line 105)
    BNO085 sensor("/dev/i2c-7", 0x4A, BNO085::ResetPin::NONE);

    sensor.initialize();
    sensor.startService();

    sensor.enableSensor(SH2_RAW_ACCELEROMETER, 100.0f);
    sensor.enableSensor(SH2_RAW_GYROSCOPE, 100.0f);
    sensor.enableSensor(SH2_RAW_MAGNETOMETER, 50.0f);
    sensor.enableSensor(SH2_GAME_ROTATION_VECTOR, 50.0f);


    BNO085::OrientationReading test_reading;

    while (true) {

        std::this_thread::sleep_for(std::chrono::seconds(1));

        sensor.getOrientationData(SH2_GAME_ROTATION_VECTOR, test_reading);
        std::cout << "Sensor ready! Initial orientation: " 
                  << test_reading.rotation.toString() << std::endl;
       
      
    }
    
    return 0;
}




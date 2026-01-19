


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












// // #include "BNO085.h"
// // #include <iostream>
// // #include <iomanip>
// // #include <signal.h>
// // #include <thread>
// // #include <chrono>

// // #include <open3d/Open3D.h>

// // int main() {
// //     // Use Pin 7 (gpiochip0 line 105)
// //     BNO085 sensor("/dev/i2c-7", 0x4A, BNO085::ResetPin::NONE);

// //     sensor.initialize();
// //     sensor.startService();



// //     while (true) {
// //         //std::this_thread::sleep_for(std::chrono::seconds(1));
// //     }
    
// //     return 0;
// // }




// main.cpp - Refactored for simplified BNO085 API
#include "BNO085.h"
#include "GUI.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <signal.h>

// =============================================================================
// GLOBAL STATE
// =============================================================================

std::atomic<bool> running{true};

void signalHandler(int signal) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
}

// =============================================================================
// SENSOR UPDATE LOOP
// =============================================================================

void sensorUpdateLoop(IMUVisualizer* visualizer, BNO085& sensor) {
    std::cout << "Sensor update loop started" << std::endl;
    
    while (running && visualizer->IsRunning()) {
        // Try to get orientation data (from whichever orientation sensor is active)
        BNO085::OrientationReading orient_data;
        BNO085::AccelerationReading accel_data;
        
        // Try different orientation sensors in priority order
        bool got_orientation = false;
        
        if (sensor.getRotationVector(orient_data)) {
            got_orientation = true;
        } else if (sensor.getGameRotationVector(orient_data)) {
            got_orientation = true;
        } else if (sensor.getGeomagneticRotationVector(orient_data)) {
            got_orientation = true;
        } else if (sensor.getStabilizedRV(orient_data)) {
            got_orientation = true;
        } else if (sensor.getStabilizedGRV(orient_data)) {
            got_orientation = true;
        }
        
        // Get acceleration data (try different sources)
        bool got_accel = false;
        if (sensor.getCalibratedAcceleration(accel_data)) {
            got_accel = true;
        } else if (sensor.getLinearAcceleration(accel_data)) {
            got_accel = true;
        } else if (sensor.getGravity(accel_data)) {
            got_accel = true;
        }
        
        // Update visualization if we have data
        if (got_orientation && got_accel) {
            visualizer->UpdateSensorData(orient_data.rotation, 
                                        accel_data.acceleration);
        } else if (got_orientation) {
            // Just orientation, use zero acceleration
            visualizer->UpdateSensorData(orient_data.rotation, 
                                        BNO085::Vector3(0, 0, 0));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }
    
    running = false;
    std::cout << "Sensor update loop ended" << std::endl;
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char* argv[]) {
    // ==========================================================================
    // CONFIGURATION - EDIT THESE VALUES
    // ==========================================================================
    const std::string MODEL_PATH = "models/bun_zipper.ply";     // Path to your 3D model
    const std::string I2C_BUS = "/dev/i2c-7";                    // I2C bus
    const uint8_t I2C_ADDRESS = 0x4A;                            // I2C address
    const BNO085::ResetPin RESET_PIN = BNO085::ResetPin::PIN_7; // Reset pin
    
    // ==========================================================================
    // SETUP
    // ==========================================================================
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "========================================" << std::endl;
    std::cout << "BNO085 IMU Visualizer (Refactored)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Model: " << MODEL_PATH << std::endl;
    std::cout << "I2C: " << I2C_BUS << " @ 0x" << std::hex 
              << static_cast<int>(I2C_ADDRESS) << std::dec << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // ==========================================================================
    // INITIALIZE SENSOR
    // ==========================================================================
    std::cout << "Initializing BNO085..." << std::endl;
    BNO085 sensor(I2C_BUS, I2C_ADDRESS, RESET_PIN);
    
    if (!sensor.initialize()) {
        std::cerr << "Failed to initialize sensor: " << sensor.getLastError() << std::endl;
        return 1;
    }
    
    // Print product info
    std::cout << "\n" << sensor.getProductInfo() << std::endl;
    
    // ==========================================================================
    // ENABLE DEFAULT SENSORS
    // ==========================================================================
    std::cout << "Enabling default sensors..." << std::endl;
    
    // Enable rotation vector (absolute orientation with magnetometer)
    // Using the super simple API - just specify frequency!
    
    // Enable basic IMU sensors for the data display tabs
    sensor.enableSensor(SH2_RAW_ACCELEROMETER, 100.0f);
    sensor.enableSensor(SH2_RAW_GYROSCOPE, 100.0f);
    sensor.enableSensor(SH2_RAW_MAGNETOMETER, 50.0f);
    sensor.enableSensor(SH2_GAME_ROTATION_VECTOR, 50.0f);

    
    
    // Optional: Enable other useful sensors
    // sensor.enableSensor(SH2_LINEAR_ACCELERATION, 100.0f);  // Accel without gravity
    // sensor.enableSensor(SH2_GRAVITY, 50.0f);               // Gravity vector only
    // sensor.enableSensor(SH2_GAME_ROTATION_VECTOR, 200.0f); // No mag, higher rate
    
    // Start the sensor service thread
    sensor.startService();
    
    // Wait for initial data
    std::cout << "Waiting for sensor data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Check if we're getting data
    BNO085::OrientationReading test_reading;
    if (sensor.getOrientationData(SH2_GAME_ROTATION_VECTOR, test_reading)) {
        std::cout << "Sensor ready! Initial orientation: " 
                  << test_reading.rotation.toString() << std::endl;
    } else {
        std::cout << "Warning: No orientation data yet, continuing anyway..." << std::endl;
    }
    
    // ==========================================================================
    // GUI SETUP
    // ==========================================================================
    
    std::cout << "\nInitializing Open3D GUI..." << std::endl;
    open3d::visualization::gui::Application::GetInstance().Initialize("/usr/local/share/Open3D/resources");
    
    // Create visualizer (pass sensor reference so GUI can control it)
    auto visualizer = std::make_shared<IMUVisualizer>(MODEL_PATH, sensor);
    visualizer->Run();
    
    // Start sensor update thread
    std::cout << "Starting sensor update thread..." << std::endl;
    std::thread update_thread(sensorUpdateLoop, visualizer.get(), std::ref(sensor));
    
    // ==========================================================================
    // RUN GUI (BLOCKING)
    // ==========================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "Starting GUI..." << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\nGUI Controls:" << std::endl;
    std::cout << "  • Select sensor type from dropdown" << std::endl;
    std::cout << "  • Adjust sampling rate (1 Hz - 400 Hz)" << std::endl;
    std::cout << "  • Enable/disable sensors with checkbox" << std::endl;
    std::cout << "  • Calibration tab: Track calibration status" << std::endl;
    std::cout << "  • Data tabs: View raw data, euler angles, quaternions" << std::endl;
    std::cout << "\nAvailable Sensors:" << std::endl;
    std::cout << "  Orientation:" << std::endl;
    std::cout << "    - Rotation Vector (with mag, absolute heading)" << std::endl;
    std::cout << "    - Game Rotation Vector (no mag, no drift)" << std::endl;
    std::cout << "    - Geomagnetic Rotation Vector" << std::endl;
    std::cout << "    - Stabilized Rotation Vector (AR/VR)" << std::endl;
    std::cout << "    - Stabilized Game Rotation (AR/VR)" << std::endl;
    std::cout << "  Motion:" << std::endl;
    std::cout << "    - Accelerometer (raw acceleration)" << std::endl;
    std::cout << "    - Linear Acceleration (no gravity)" << std::endl;
    std::cout << "    - Gravity (gravity vector only)" << std::endl;
    std::cout << "  Angular:" << std::endl;
    std::cout << "    - Gyroscope (calibrated/uncalibrated)" << std::endl;
    std::cout << "  Magnetic:" << std::endl;
    std::cout << "    - Magnetometer (calibrated/uncalibrated)" << std::endl;
    std::cout << "\n========================================\n" << std::endl;
    
    // Run the GUI main loop
    open3d::visualization::gui::Application::GetInstance().Run();
    
    // ==========================================================================
    // CLEANUP
    // ==========================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "Cleaning up..." << std::endl;
    std::cout << "========================================" << std::endl;
    
    running = false;
    update_thread.join();
    
    sensor.stopService();
    sensor.shutdown();
    
    std::cout << "\nDone! Goodbye." << std::endl;
    return 0;
}
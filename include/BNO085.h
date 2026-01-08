// BNO085.h
#ifndef BNO085_H
#define BNO085_H

#include "sh2_c_interface.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <gpiod.h>

class BNO085 {
public:
    // ==========================================================================
    // DATA STRUCTURES
    // ==========================================================================
    
    struct Vector3 {
        float x, y, z;
        
        Vector3() : x(0), y(0), z(0) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        
        // Utility methods
        float magnitude() const;
        Vector3 normalized() const;
        std::string toString() const;
    };
    

    struct Quaternion {
        float w, x, y, z;
        float accuracy;  // Accuracy estimate in radians (for rotation vector)
        
        Quaternion() : w(1), x(0), y(0), z(0), accuracy(0) {}
        Quaternion(float w, float x, float y, float z, float acc = 0) 
            : w(w), x(x), y(y), z(z), accuracy(acc) {}
        
        // Utility methods
        Vector3 toEulerAngles() const;  // Returns yaw, pitch, roll in degrees
        std::string toString() const;
    };
    
    struct SensorReading {
        uint64_t timestamp_us;
        uint8_t status;        // 0=unreliable, 1=low, 2=medium, 3=high accuracy
        uint8_t sequence;
        
        SensorReading() : timestamp_us(0), status(0), sequence(0) {}
    };
    
   
    struct IMUData : public SensorReading {
        Vector3 acceleration;      // m/s²
        Vector3 gyroscope;         // rad/s
        Vector3 magnetometer;      // µT
        
        IMUData() = default;
    };
    
    
    struct OrientationData : public SensorReading {
        Quaternion rotation;
        Vector3 euler;  // Yaw, pitch, roll in degrees (computed from quaternion)
        
        OrientationData() = default;
    };
    
    
    struct SensorConfig {
        uint32_t reportInterval_us;     // Report interval in microseconds
        bool enabled;                   // Whether sensor is enabled
        bool wakeupEnabled;             // Wake system on sensor event
        bool alwaysOnEnabled;           // Keep sensor on during sleep
        
        SensorConfig() : reportInterval_us(10000), enabled(false), 
                        wakeupEnabled(false), alwaysOnEnabled(false) {}
        
        // Convenience methods
        void setFrequency(float hz) { 
            reportInterval_us = static_cast<uint32_t>(1000000.0f / hz); 
        }
        float getFrequency() const { 
            return 1000000.0f / reportInterval_us; 
        }
    };
    
    // ==========================================================================
    // CALLBACK TYPES
    // ==========================================================================
    
    using IMUCallback = std::function<void(const IMUData& data)>;
    using OrientationCallback = std::function<void(const OrientationData& data)>;
    using SensorEventCallback = std::function<void(sh2_SensorId_t sensorId, const sh2_SensorValue_t& value)>;
    using ErrorCallback = std::function<void(const std::string& error)>;
    
    // ==========================================================================
    // ENUMS
    // ==========================================================================
    
    enum class CalibrationStatus {
        UNCALIBRATED = 0,
        LOW_ACCURACY = 1,
        MEDIUM_ACCURACY = 2,
        HIGH_ACCURACY = 3
    };
    
    enum class SensorType {
        ACCELEROMETER,
        GYROSCOPE, 
        MAGNETOMETER,
        ROTATION_VECTOR,
        GAME_ROTATION_VECTOR,
        LINEAR_ACCELERATION,
        GRAVITY
    };

    enum class ResetPin {
    NONE = -1,          // No hardware reset
    PIN_7  = 105,       // Physical pin 7  (PQ.05) - BEST CHOICE
    PIN_11 = 112,       // Physical pin 11 (PR.04)
    PIN_13 = 113,       // Physical pin 13 (PR.05)
    PIN_15 = 49,        // Physical pin 15 (PH.06)
    PIN_16 = 89,        // Physical pin 16 (PN.05)
    PIN_18 = 88,        // Physical pin 18 (PN.04)
    PIN_22 = 90,        // Physical pin 22 (PN.06)
    PIN_29 = 96,        // Physical pin 29 (PP.04)
    PIN_31 = 97,        // Physical pin 31 (PP.05)
    PIN_32 = 98,        // Physical pin 32 (PP.06)
    PIN_33 = 99,        // Physical pin 33 (PP.07)
    PIN_36 = 91,        // Physical pin 36 (PN.07)
    PIN_37 = 86,        // Physical pin 37 (PN.02)
    
};
    
    // ==========================================================================
    // CONSTRUCTOR/DESTRUCTOR
    // ==========================================================================
    
 
    BNO085();
    BNO085(const std::string& i2c_bus, uint8_t i2c_address, ResetPin reset_pin = ResetPin::NONE);
    ~BNO085();
    
    // Prevent copying
    BNO085(const BNO085&) = delete;
    BNO085& operator=(const BNO085&) = delete;
    

    // =============================================================================
    // GPIO HELPER METHODS
    // =============================================================================
    
    bool hardwareReset();
    bool isValidResetPin(ResetPin pin) const;
    bool initializeGPIO();
    void releaseGPIO();

    // ==========================================================================
    // INITIALIZATION AND CONTROL
    // ==========================================================================
    
    bool initialize();
    void shutdown();
    bool isInitialized() const { return initialized_; }
    void startService();
    void stopService();
    bool isServiceRunning() const { return service_running_; }
    
    
    // ==========================================================================
    // SENSOR CONFIGURATION
    // ==========================================================================
    
    bool enableSensor(SensorType sensor, const SensorConfig& config);
    bool disableSensor(SensorType sensor);
    bool enableBasicIMU();
    bool enableOrientation();
    bool getSensorConfig(SensorType sensor, SensorConfig& config);
    
    // ==========================================================================
    // DATA READING
    // ==========================================================================
  
    bool getAcceleration(Vector3& data);
    bool getGyroscope(Vector3& data);
    bool getMagnetometer(Vector3& data);
    bool getOrientation(Quaternion& data);
    bool getEulerAngles(float& yaw, float& pitch, float& roll);
    bool getIMUData(IMUData& data);
    bool getOrientationData(OrientationData& data);
    
    // ==========================================================================
    // CALIBRATION
    // ==========================================================================
    
   
    void getCalibrationStatus(CalibrationStatus& accel, CalibrationStatus& gyro, 
                             CalibrationStatus& mag, CalibrationStatus& system);
    bool isFullyCalibrated();
    bool startCalibration(uint32_t interval_us = 10000);
    bool finishCalibration();
    bool saveCalibration();
    
    // ==========================================================================
    // CALLBACKS
    // ==========================================================================
    
    void setIMUCallback(IMUCallback callback) { imu_callback_ = callback; }
    void setOrientationCallback(OrientationCallback callback) { orientation_callback_ = callback; }
    void setSensorEventCallback(SensorEventCallback callback) { sensor_callback_ = callback; }
    void setErrorCallback(ErrorCallback callback) { error_callback_ = callback; }
    
    // ==========================================================================
    // DIAGNOSTICS AND STATUS
    // ==========================================================================
    
    std::string getProductInfo();
    std::string getLastError() const { return last_error_; }
    bool isConnected();
    bool reset();
    bool wasReset();
    std::string getSensorMetadata(SensorType sensor);

private:
    // ==========================================================================
    // PRIVATE MEMBERS
    // ==========================================================================
    
    // Hardware interface
    sh2_Hal_t* hal_;
    std::string i2c_bus_;
    uint8_t i2c_address_;
    ResetPin reset_pin_;
    gpiod_chip* gpio_chip_;
    gpiod_line* gpio_line_;
    static constexpr const char* GPIO_CHIP_NAME = "gpiochip0";  // Jetson default
    
    // State management
    std::atomic<bool> initialized_;
    std::atomic<bool> service_running_;
    std::string last_error_;
    std::atomic<bool> reset_occurred_;
    
    // Service thread
    std::unique_ptr<std::thread> service_thread_;
    
    // Data storage with thread-safe access
    mutable std::mutex data_mutex_;
    IMUData latest_imu_;
    OrientationData latest_orientation_;
    std::map<sh2_SensorId_t, sh2_SensorValue_t> latest_sensor_data_;
    
    // Callbacks
    IMUCallback imu_callback_;
    OrientationCallback orientation_callback_;
    SensorEventCallback sensor_callback_;
    ErrorCallback error_callback_;
    
    // ==========================================================================
    // PRIVATE METHODS
    // ==========================================================================
    
    
    void setError(const std::string& error);
    sh2_SensorId_t sensorTypeToId(SensorType sensor);
    void serviceLoop();
    
    // SH2 library callback handlers (static functions)
    static void eventCallbackWrapper(void* cookie, sh2_AsyncEvent_t* event);
    static void sensorCallbackWrapper(void* cookie, sh2_SensorEvent_t* event);
    
    // Instance callback handlers
    void handleAsyncEvent(sh2_AsyncEvent_t* event);
    void handleSensorEvent(sh2_SensorEvent_t* event);
    
    // Data processing
    void updateIMUData(const sh2_SensorValue_t& value);
    void updateOrientationData(const sh2_SensorValue_t& value);
};

#endif // BNO085_H
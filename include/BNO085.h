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
#include <cstring>
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

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
        bool has_accuracy;
        Quaternion() : w(1), x(0), y(0), z(0), accuracy(0), has_accuracy(false) {}
        Quaternion(float w, float x, float y, float z, float acc = -1) 
            : w(w), x(x), y(y), z(z), accuracy(acc), has_accuracy(acc >= 0) {}
        
        // Utility methods
        Vector3 toEulerAngles() const;  // Returns yaw, pitch, roll in degrees
        std::string toString() const;
    };

    // ==========================================================================
    // ENUMS
    // ==========================================================================

    
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
    // SENSOR READINGS
    // ==========================================================================
    


    struct Metadata  { //Represents Common header 
        sh2_SensorId_t sensor_id;
        uint8_t sequence;               // Sequence number
        std::string status;                 // Calibration quality
        uint32_t delay;                 // Report delivery delay
        uint32_t report_timestamp;      // Sensor timestamp in microseconds

        
        Metadata() 
            : sensor_id(SH2_MAX_SENSOR_ID), delay(0), sequence(0), 
              status("UNRELIABLE"), report_timestamp(0) {}
    };

    struct SensorReading {
        Metadata meta;
        SensorReading() : meta() {}

        std::string getSensorIdString() const { return sensorIdToString(meta.sensor_id); }
    };

    struct AccelerationReading : public SensorReading {
        Vector3 acceleration;  // Acceleration in m/s²
        uint32_t sensor_timestamp; // Sensor timestamp in microseconds
        bool has_timestamp;

        AccelerationReading() : has_timestamp(false), sensor_timestamp(0) {}
    };

    struct AngularVelocityReading : public SensorReading {
        Vector3 angular_velocity;  // Angular velocity in rad/s
        Vector3 bias;              // Only valid for uncalibrated gyro
        bool has_bias;
        uint32_t sensor_timestamp; // Sensor timestamp in microseconds
        bool has_timestamp;

        AngularVelocityReading() : has_bias(false), has_timestamp(false), sensor_timestamp(0) {}
    };

    struct MagneticFieldReading : public SensorReading {
        Vector3 magnetic_field;  // Magnetic field in µT
        Vector3 bias;            // Only valid for uncalibrated mag
        bool has_bias;
        uint32_t sensor_timestamp; // Sensor timestamp in microseconds
        bool has_timestamp;
        
        MagneticFieldReading() : has_bias(false), has_timestamp(false), sensor_timestamp(0) {}
    };

    struct OrientationReading : public SensorReading {
        Quaternion rotation;
        Vector3 euler;      // Computed from quaternion (yaw, pitch, roll in degrees)        
        Vector3 angular_velocity; // Optional angular velocity in rad/s
        bool has_angular_velocity;
        OrientationReading() : has_angular_velocity(false) {}
    };
    
    // ==========================================================================
    // CALLBACK TYPES
    // ==========================================================================
    
    using AccelerationCallback = std::function<void(const AccelerationReading& data)>;
    using AngularVelocityCallback = std::function<void(const AngularVelocityReading& data)>;
    using MagneticFieldCallback = std::function<void(const MagneticFieldReading& data)>;
    using OrientationCallback = std::function<void(const OrientationReading& data)>;
    using SensorEventCallback = std::function<void(sh2_SensorId_t sensorId, const sh2_SensorValue_t& value)>;
    using ErrorCallback = std::function<void(const std::string& error)>;

    // ==========================================================================
    // CONSTRUCTOR/DESTRUCTOR
    // ==========================================================================
    
    BNO085();
    BNO085(const std::string& i2c_bus, uint8_t i2c_address, ResetPin reset_pin = ResetPin::NONE);
    ~BNO085();
    
    // Prevent copying
    BNO085(const BNO085&) = delete;
    BNO085& operator=(const BNO085&) = delete;

    // ==========================================================================
    // GPIO HELPER METHODS
    // ==========================================================================
    
    bool hardwareReset();
    bool isValidResetPin(ResetPin pin) const;
    bool initializeGPIO();
    void releaseGPIO();

    // ==========================================================================
    // SERVICE METHODS
    // ==========================================================================
    
    bool initialize();
    void shutdown();
    bool isInitialized() const { return initialized_; }
    void startService();
    void stopService();
    bool isServiceRunning() const { return service_running_; }
    
    // ==========================================================================
    // SENSOR CONTROL
    // ==========================================================================
    
    // Helper to create sh2 config with sensible defaults
    static sh2_SensorConfig_t createSensorConfig(float frequency_hz, 
                                                  bool wakeup = false, 
                                                  bool always_on = false) {

                                                    
        sh2_SensorConfig_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.reportInterval_us = static_cast<uint32_t>(1000000.0f / frequency_hz);
        cfg.wakeupEnabled = wakeup;
        cfg.alwaysOnEnabled = always_on;
        cfg.changeSensitivityEnabled = false;
        cfg.changeSensitivityRelative = false;
        cfg.changeSensitivity = 0;
        cfg.batchInterval_us = 0;
        cfg.sensorSpecific = 0;
        return cfg;
    }

    static sh2_SensorConfig_t createSensorConfig(float frequency_hz, 
                                                  bool wakeup, 
                                                  bool always_on,
                                                  bool change_sensitivity_enabled,
                                                  bool change_sensitivity_relative,
                                                  uint16_t change_sensitivity,
                                                  uint32_t batch_interval_us,
                                                  uint32_t sensor_specific ) {

                                                    
        sh2_SensorConfig_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.reportInterval_us = static_cast<uint32_t>(1000000.0f / frequency_hz);
        cfg.wakeupEnabled = wakeup;
        cfg.alwaysOnEnabled = always_on;
        cfg.changeSensitivityEnabled = change_sensitivity_enabled;
        cfg.changeSensitivityRelative = change_sensitivity_relative;
        cfg.changeSensitivity = change_sensitivity;
        cfg.batchInterval_us = batch_interval_us;
        cfg.sensorSpecific = sensor_specific;
        return cfg;
    }


    
    // Primary API - uses sh2 config directly (full control)
    bool enableSensor(sh2_SensorId_t sensor_id, const sh2_SensorConfig_t& config);
    
    // Simple overload - just specify frequency
    bool enableSensor(sh2_SensorId_t sensor_id, float frequency_hz) {
        if (MAX_FREQUENCIES.find(sensor_id) != MAX_FREQUENCIES.end()) {
            float max_freq = MAX_FREQUENCIES.at(sensor_id);
            if (frequency_hz > max_freq) {
                frequency_hz = max_freq;  // Cap to max supported frequency
            }
        }
        return enableSensor(sensor_id, createSensorConfig(frequency_hz));
    }
    
    bool disableSensor(sh2_SensorId_t sensor_id);
    bool getSensorConfig(sh2_SensorId_t sensor_id, sh2_SensorConfig_t& config);
    static const std::map<sh2_SensorId_t, float> MAX_FREQUENCIES;
        
    // ==========================================================================
    // DATA READING
    // ==========================================================================
    
    // Generic getters by sensor ID
    bool getAccelerationData(sh2_SensorId_t sensor_id, AccelerationReading& data);
    bool getAngularVelocityData(sh2_SensorId_t sensor_id, AngularVelocityReading& data);
    bool getMagneticFieldData(sh2_SensorId_t sensor_id, MagneticFieldReading& data);
    bool getOrientationData(sh2_SensorId_t sensor_id, OrientationReading& data);

    // Convenience methods for commonly used sensors
    bool getLinearAcceleration(AccelerationReading& data) {
        return getAccelerationData(SH2_LINEAR_ACCELERATION, data);
    }
    
    bool getGravity(AccelerationReading& data) {
        return getAccelerationData(SH2_GRAVITY, data);
    }
    
    bool getCalibratedAcceleration(AccelerationReading& data) {
        return getAccelerationData(SH2_ACCELEROMETER, data);
    }
    
    bool getCalibratedGyroscope(AngularVelocityReading& data) {
        return getAngularVelocityData(SH2_GYROSCOPE_CALIBRATED, data);
    }
    
    bool getUncalibratedGyroscope(AngularVelocityReading& data) {
        return getAngularVelocityData(SH2_GYROSCOPE_UNCALIBRATED, data);
    }
    
    bool getCalibratedMagnetometer(MagneticFieldReading& data) {
        return getMagneticFieldData(SH2_MAGNETIC_FIELD_CALIBRATED, data);
    }
    
    bool getUncalibratedMagnetometer(MagneticFieldReading& data) {
        return getMagneticFieldData(SH2_MAGNETIC_FIELD_UNCALIBRATED, data);
    }
    
    bool getRotationVector(OrientationReading& data) {
        return getOrientationData(SH2_ROTATION_VECTOR, data);
    }
    
    bool getGameRotationVector(OrientationReading& data) {
        return getOrientationData(SH2_GAME_ROTATION_VECTOR, data);
    }
    
    bool getGeomagneticRotationVector(OrientationReading& data) {
        return getOrientationData(SH2_GEOMAGNETIC_ROTATION_VECTOR, data);
    }
    
    bool getStabilizedRV(OrientationReading& data) {
        return getOrientationData(SH2_ARVR_STABILIZED_RV, data);
    }
    
    bool getStabilizedGRV(OrientationReading& data) {
        return getOrientationData(SH2_ARVR_STABILIZED_GRV, data);
    }

    // ==========================================================================
    // CALIBRATION
    // ==========================================================================
       // Calibration methods
    bool enableDynamicCalibration(bool accel, bool gyro, bool mag);
    bool saveCalibration();
    bool clearCalibration();
    // CalibrationStatus getCalibrationStatus();
    
    // Simple calibration
    bool startManualCalibration();
    sh2_CalStatus_t finishManualCalibration();
    
    // Taring methods
    bool tareAllAxes();
    bool tareZAxis();
    bool clearTare();
    bool persistTare();
    bool setCustomOrientation(const Quaternion& orientation);
        
    // ==========================================================================
    // CALLBACKS
    // ==========================================================================
    
    void setAccelerationCallback(AccelerationCallback callback) { accel_callback_ = callback; }
    void setAngularVelocityCallback(AngularVelocityCallback callback) { angular_vel_callback_ = callback; }
    void setMagneticFieldCallback(MagneticFieldCallback callback) { mag_field_callback_ = callback; }
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
    std::string getSensorMetadata(sh2_SensorId_t sensor_id);

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
    std::map<sh2_SensorId_t, AccelerationReading> latest_acceleration_;
    std::map<sh2_SensorId_t, AngularVelocityReading> latest_angular_velocity_;
    std::map<sh2_SensorId_t, MagneticFieldReading> latest_magnetic_field_;
    std::map<sh2_SensorId_t, OrientationReading> latest_orientation_;

    // Callbacks - category-wide
    AccelerationCallback accel_callback_;
    AngularVelocityCallback angular_vel_callback_;
    MagneticFieldCallback mag_field_callback_;
    OrientationCallback orientation_callback_;
    SensorEventCallback sensor_callback_;
    ErrorCallback error_callback_;
    
    // Callbacks - sensor-specific
    std::map<sh2_SensorId_t, AccelerationCallback> accel_callbacks_;
    std::map<sh2_SensorId_t, AngularVelocityCallback> angular_vel_callbacks_;
    std::map<sh2_SensorId_t, MagneticFieldCallback> mag_field_callbacks_;
    std::map<sh2_SensorId_t, OrientationCallback> orientation_callbacks_;

    std::map<sh2_SensorId_t, sh2_SensorConfig_t> enabled_sensors_;
    
    // ==========================================================================
    // PRIVATE METHODS
    // ==========================================================================
    
    void setError(const std::string& error);
    void serviceLoop();
    
    // SH2 library callback handlers (static functions)
    static void eventCallbackWrapper(void* cookie, sh2_AsyncEvent_t* event);
    static void sensorCallbackWrapper(void* cookie, sh2_SensorEvent_t* event);
    
    // Instance callback handlers
    void handleAsyncEvent(sh2_AsyncEvent_t* event);
    void handleShtpError(sh2_ShtpEvent_t shtpEvent);
    void handleFeatureResponse(const sh2_SensorConfigResp_t& resp);
    void handleResetEvent();
    //Continous Response
    void handleSensorEvent(sh2_SensorEvent_t* event);

    

    // Data processing - category-specific update methods
    void updateAccelerationData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value);
    void updateAngularVelocityData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value);
    void updateMagneticFieldData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value);
    void updateOrientationData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value);
};

#endif // BNO085_H
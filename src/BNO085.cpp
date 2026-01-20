// BNO085.cpp - Refactored to use sh2 types directly
#include "BNO085.h"
#include "I2C_HAL.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <chrono>

// =============================================================================
// UTILITY IMPLEMENTATIONS
// =============================================================================

float BNO085::Vector3::magnitude() const {
    return std::sqrt(x*x + y*y + z*z);
}

BNO085::Vector3 BNO085::Vector3::normalized() const {
    float mag = magnitude();
    if (mag > 0) {
        return Vector3(x/mag, y/mag, z/mag);
    }
    return Vector3();
}

std::string BNO085::Vector3::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) 
        << "(" << x << ", " << y << ", " << z << ")";
    return oss.str();
}

BNO085::Vector3 BNO085::Quaternion::toEulerAngles() const {
    float yaw, pitch, roll;
    q_to_ypr(w, x, y, z, &yaw, &pitch, &roll);
    
    // Convert from radians to degrees
    return Vector3(yaw * 180.0f / M_PI, 
                   pitch * 180.0f / M_PI, 
                   roll * 180.0f / M_PI);
}

std::string BNO085::Quaternion::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) 
        << "w=" << w << " x=" << x << " y=" << y << " z=" << z;
    if (accuracy > 0) {
        oss << " (acc=" << accuracy << ")";
    }
    return oss.str();
}

const std::map<sh2_SensorId_t, float> BNO085::MAX_FREQUENCIES = {
    // Orientation sensors
    {SH2_ROTATION_VECTOR, 400.0f},                  // Max 400 Hz
    {SH2_GAME_ROTATION_VECTOR, 400.0f},             // Max 400 Hz
    {SH2_GEOMAGNETIC_ROTATION_VECTOR, 90.0f},       // Max 90 Hz (limited by mag)
    {SH2_ARVR_STABILIZED_RV, 400.0f},               // Max 400 Hz
    {SH2_ARVR_STABILIZED_GRV, 400.0f},              // Max 400 Hz
    {SH2_GYRO_INTEGRATED_RV, 400.0f},               // Max 400 Hz
    
    // Physical sensors
    {SH2_ACCELEROMETER, 500.0f},                    // Max 500 Hz
    {SH2_GYROSCOPE_CALIBRATED, 1000.0f},            // Max 1000 Hz
    {SH2_GYROSCOPE_UNCALIBRATED, 1000.0f},          // Max 1000 Hz
    {SH2_MAGNETIC_FIELD_CALIBRATED, 100.0f},        // Max 100 Hz
    {SH2_MAGNETIC_FIELD_UNCALIBRATED, 100.0f},      // Max 100 Hz
    {SH2_LINEAR_ACCELERATION, 400.0f},              // Max 400 Hz
    {SH2_GRAVITY, 400.0f},                          // Max 400 Hz
    
    // Raw sensors
    {SH2_RAW_ACCELEROMETER, 500.0f},                // Max 500 Hz
    {SH2_RAW_GYROSCOPE, 1000.0f},                   // Max 1000 Hz
    {SH2_RAW_MAGNETOMETER, 100.0f},                 // Max 100 Hz
};

// =============================================================================
// CONSTRUCTOR/DESTRUCTOR
// =============================================================================

BNO085::BNO085() 
    : BNO085("/dev/i2c-1", 0x4A) {
}

BNO085::BNO085(const std::string& i2c_bus, uint8_t i2c_address, ResetPin reset_pin)
    : hal_(nullptr)
    , i2c_bus_(i2c_bus)
    , i2c_address_(i2c_address)
    , reset_pin_(reset_pin)
    , gpio_chip_(nullptr)
    , gpio_line_(nullptr)
    , initialized_(false)
    , service_running_(false) 
    , reset_occurred_(false) {

    if (!isValidResetPin(reset_pin)) {
        std::cerr << "Warning: Invalid reset pin specified. Hardware reset disabled." << std::endl;
        reset_pin_ = ResetPin::NONE;
    }
}

BNO085::~BNO085() {
    shutdown();
}

// =============================================================================
// GPIO METHODS
// =============================================================================

bool BNO085::isValidResetPin(ResetPin pin) const {
    // Check if pin is in valid range (not NONE)
    int pin_value = static_cast<int>(pin);
    return pin_value >= 0;  // All enum values >= 0 are valid GPIO numbers
}

bool BNO085::initializeGPIO() {
    if (reset_pin_ == ResetPin::NONE) {
        return false;  // No GPIO configured
    }
    
    // Open GPIO chip
    gpio_chip_ = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
    if (!gpio_chip_) {
        setError("Failed to open GPIO chip: " + std::string(GPIO_CHIP_NAME));
        return false;
    }
    
    // Get GPIO line
    unsigned int line_offset = static_cast<unsigned int>(reset_pin_);
    gpio_line_ = gpiod_chip_get_line(gpio_chip_, line_offset);
    if (!gpio_line_) {
        setError("Failed to get GPIO line " + std::to_string(line_offset));
        gpiod_chip_close(gpio_chip_);
        gpio_chip_ = nullptr;
        return false;
    }
    
    // Request line as output, starting HIGH (inactive reset)
    int ret = gpiod_line_request_output(gpio_line_, "BNO085-RST", 1);
    if (ret < 0) {
        setError("Failed to request GPIO line as output");
        gpiod_chip_close(gpio_chip_);
        gpio_chip_ = nullptr;
        gpio_line_ = nullptr;
        return false;
    }
    
    return true;
}

void BNO085::releaseGPIO() {
    if (gpio_line_) {
        gpiod_line_release(gpio_line_);
        gpio_line_ = nullptr;
    }
    
    if (gpio_chip_) {
        gpiod_chip_close(gpio_chip_);
        gpio_chip_ = nullptr;
    }
}

bool BNO085::hardwareReset() {
    if (!gpio_line_) {
        setError("No GPIO configured for hardware reset");
        return false;
    }
    
    std::cout << "Performing hardware reset..." << std::endl;
    
    // Step 1: Stop service thread if running
    bool was_service_running = service_running_;
    if (was_service_running) {
        std::cout << "Stopping service thread for reset..." << std::endl;
        stopService();
    }
    
    // Step 2: Close SH2 library to clean up state
    if (initialized_) {
        std::cout << "Closing SH2 library..." << std::endl;
        sh2_close();
    }
    
    // Step 3: Perform hardware reset sequence
    // Reset sequence: HIGH -> LOW (100ms) -> HIGH
    
    // Ensure we start HIGH
    if (gpiod_line_set_value(gpio_line_, 1) < 0) {
        setError("Failed to set GPIO HIGH");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Pull LOW (activate reset)
    if (gpiod_line_set_value(gpio_line_, 0) < 0) {
        setError("Failed to set GPIO LOW");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Release reset (back to HIGH)
    if (gpiod_line_set_value(gpio_line_, 1) < 0) {
        setError("Failed to set GPIO HIGH");
        return false;
    }
    
    std::cout << "Hardware reset complete, waiting for device to stabilize..." << std::endl;
    
    // Step 4: Wait for device to fully boot
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Step 5: Reinitialize SH2 library
    if (initialized_) {
        std::cout << "Reinitializing SH2 library..." << std::endl;
        int result = sh2_open(hal_, eventCallbackWrapper, this);
        if (result != SH2_OK) {
            setError("Failed to reopen SH2 library after reset: " + sh2ErrorToString(result));
            return false;
        }
        
        // Re-register sensor callback
        sh2_setSensorCallback(sensorCallbackWrapper, this);
        
        // Wait a moment for initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Step 6: Restart service thread if it was running
    if (was_service_running) {
        std::cout << "Restarting service thread..." << std::endl;
        startService();
    }
    
    std::cout << "Hardware reset and reinitialization complete" << std::endl;
    
    return true;
}

// =============================================================================
// INITIALIZATION AND CONTROL
// =============================================================================

bool BNO085::initialize() {
    if (initialized_) {
        return true;
    }
    
    std::cout << "Initializing BNO085 sensor..." << std::endl;

    // Initialize GPIO if reset pin configured
    if (reset_pin_ != ResetPin::NONE) {
        if (initializeGPIO()) {
        
        } else {
            std::cerr << "Warning: GPIO initialization failed. Hardware reset unavailable." << std::endl;
        }
    }
    
    // Create HAL
    hal_ = createI2CHAL(i2c_bus_.c_str(), i2c_address_);
    if (!hal_) {
        setError("Failed to create I2C HAL");
        return false;
    }
    
    // Open SH2 library
    int result = sh2_open(hal_, eventCallbackWrapper, this);
    if (result != SH2_OK) {
        setError("Failed to open SH2 library: " + sh2ErrorToString(result));
        destroyI2CHAL(hal_);
        hal_ = nullptr;
        return false;
    }
    
    // Set sensor callback
    sh2_setSensorCallback(sensorCallbackWrapper, this);
    
    initialized_ = true;
    std::cout << "BNO085 initialized successfully" << std::endl;
    
    return true;
}

void BNO085::shutdown() {
    stopService();
    
    if (initialized_) {
        sh2_close();
        initialized_ = false;
    }
    
    if (hal_) {
        destroyI2CHAL(hal_);
        hal_ = nullptr;
    }
    
    releaseGPIO();
    
    std::cout << "BNO085 shutdown complete" << std::endl;
}

void BNO085::startService() {
    if (service_running_) {
        return;
    }
    
    service_running_ = true;
    service_thread_ = std::make_unique<std::thread>(&BNO085::serviceLoop, this);
}

void BNO085::stopService() {
    if (!service_running_) {
        return;
    }
    
    service_running_ = false;
    
    if (service_thread_ && service_thread_->joinable()) {
        service_thread_->join();
    }
    service_thread_.reset();
}

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================

bool BNO085::enableSensor(sh2_SensorId_t sensor_id, const sh2_SensorConfig_t& config) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    int result = sh2_setSensorConfig(sensor_id, &config);
    if (result != SH2_OK) {
        setError("Failed to enable sensor " + sensorIdToString(sensor_id) + ": " + sh2ErrorToString(result));
        return false;
    }
    enabled_sensors_[sensor_id] = config;
    std::cout << "Enabled sensor: " << sensorIdToString(sensor_id) << std::endl;
    
    return true;
}

bool BNO085::disableSensor(sh2_SensorId_t sensor_id) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.reportInterval_us = 0;  // 0 = disable
    
    int result = sh2_setSensorConfig(sensor_id, &config);
    if (result != SH2_OK) {
        setError("Failed to disable sensor: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cout << "Disabled sensor: " << sensorIdToString(sensor_id) << std::endl;
    return true;
}

bool BNO085::getSensorConfig(sh2_SensorId_t sensor_id, sh2_SensorConfig_t& config) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    int result = sh2_getSensorConfig(sensor_id, &config);
    if (result != SH2_OK) {
        setError("Failed to get sensor config: " + sh2ErrorToString(result));
        return false;
    }
    
    return true;
}

// =============================================================================
// DATA READING by sensor ID
// =============================================================================

bool BNO085::getAccelerationData(sh2_SensorId_t sensor_id, AccelerationReading& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_acceleration_.find(sensor_id);
    if (it == latest_acceleration_.end()) {
        return false;  // This sensor hasn't been enabled or no data yet
    }

    
    data = it->second;
    return true;
}

bool BNO085::getAngularVelocityData(sh2_SensorId_t sensor_id, AngularVelocityReading& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_angular_velocity_.find(sensor_id);
    if (it == latest_angular_velocity_.end()) {
        return false;
    }
    
    data = it->second;
    return true;
}

bool BNO085::getMagneticFieldData(sh2_SensorId_t sensor_id, MagneticFieldReading& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_magnetic_field_.find(sensor_id);
    if (it == latest_magnetic_field_.end()) {
        return false;
    }

    
    data = it->second;
    return true;
}

bool BNO085::getOrientationData(sh2_SensorId_t sensor_id, OrientationReading& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto it = latest_orientation_.find(sensor_id);
    if (it == latest_orientation_.end()) {
        return false;
    }

    
    data = it->second;
    return true;
}

// =============================================================================
// CALIBRATION
// =============================================================================


bool BNO085::enableDynamicCalibration(bool accel, bool gyro, bool mag) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    // Build calibration configuration flags
    uint8_t config = 0;
    
    if (accel) {
        config |= SH2_CAL_ACCEL;
    }
    if (gyro) {
        config |= SH2_CAL_GYRO;
    }
    if (mag) {
        config |= SH2_CAL_MAG;
    }
    
    // Set the calibration configuration
    int result = sh2_setCalConfig(config);
    if (result != SH2_OK) {
        setError("Failed to configure dynamic calibration: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cout << "Dynamic calibration configured: "
              << (accel ? "Accel " : "")
              << (gyro ? "Gyro " : "")
              << (mag ? "Mag" : "") << std::endl;
    
    // Enable automatic DCD saving every 5 seconds
    result = sh2_setDcdAutoSave(true);
    if (result != SH2_OK) {
        setError("Failed to enable auto-save: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cout << "Automatic calibration saving enabled (every 5 seconds)" << std::endl;
    
    return true;
}

bool BNO085::saveCalibration() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Saving dynamic calibration data to flash..." << std::endl;
    
    int result = sh2_saveDcdNow();
    if (result != SH2_OK) {
        setError("Failed to save calibration: " + sh2ErrorToString(result));
        return false;
    }
    
    // Give the sensor time to complete the flash write
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Calibration data saved successfully" << std::endl;
    return true;
}

bool BNO085::clearCalibration() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Clearing calibration data and resetting sensor..." << std::endl;
    
    // Stop service thread before reset
    bool was_running = service_running_;
    if (was_running) {
        stopService();
    }
    
    // Clear DCD and reset (atomic operation)
    int result = sh2_clearDcdAndReset();
    if (result != SH2_OK) {
        setError("Failed to clear calibration: " + sh2ErrorToString(result));
        if (was_running) {
            startService();
        }
        return false;
    }
    
    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Restart service if it was running
    if (was_running) {
        startService();
    }
    
    std::cout << "Calibration cleared and sensor reset" << std::endl;
    return true;
}

bool BNO085::startManualCalibration() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Starting manual calibration procedure..." << std::endl;
    std::cout << "Instructions:" << std::endl;
    std::cout << "  1. Keep sensor stationary for 2 seconds" << std::endl;
    std::cout << "  2. Rotate sensor 180 degrees smoothly" << std::endl;
    std::cout << "  3. Keep sensor stationary for 2 seconds" << std::endl;
    std::cout << "  4. Call finishManualCalibration() when done" << std::endl;
    
    // Start calibration with 10ms report interval (100 Hz)
    uint32_t interval_us = 10000;
    int result = sh2_startCal(interval_us);
    
    if (result != SH2_OK) {
        setError("Failed to start calibration: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cout << "Calibration started. Waiting for sensor to be ready..." << std::endl;
    
    // Give sensor time to prepare
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Sensor ready. Begin 180-degree rotation now." << std::endl;
    
    return true;
}

sh2_CalStatus_t BNO085::finishManualCalibration() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return sh2_CalStatus_t::SH2_CAL_NO_STATIONARY_DETECTION;
    }
    
    std::cout << "Finishing manual calibration..." << std::endl;
    
    sh2_CalStatus_t status;
    int result = sh2_finishCal(&status);
    
    if (result != SH2_OK) {
        setError("Failed to finish calibration: " + sh2ErrorToString(result));
        return status;
    }
    
    // Report calibration result
    std::cout << "Calibration result: " << calibrationStatusToString(status) << std::endl;
    
    if (status == sh2_CalStatus_t::SH2_CAL_SUCCESS) {
        std::cout << "SUCCESS! Calibration data saved to flash." << std::endl;
        std::cout << "IMPORTANT: Sensor must be reset for new calibration to take effect." << std::endl;
        std::cout << "Call reset() or perform a hardware reset." << std::endl;
    } else {
        std::cout << "FAILED! Calibration unsuccessful." << std::endl;
        std::cout << "Try again, ensuring:" << std::endl;
        std::cout << "  - Sensor is held steady at start and end" << std::endl;
        std::cout << "  - Rotation is smooth and approximately 180 degrees" << std::endl;
        std::cout << "  - Rotation takes 2-5 seconds" << std::endl;
    }
    
    return status;
}

bool BNO085::tareAllAxes() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Taring all axes (X, Y, Z)..." << std::endl;
    std::cout << "Current orientation will become the new reference frame." << std::endl;
    
    // Tare all three axes using rotation vector as basis
    int result = sh2_setTareNow(SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z, 
                                SH2_TARE_BASIS_ROTATION_VECTOR);
    
    if (result != SH2_OK) {
        setError("Failed to tare all axes: " + sh2ErrorToString(result));
        return false;
    }
    
    // Give sensor time to apply the tare
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "All axes tared successfully." << std::endl;
    std::cout << "Use persistTare() to save this tare to flash." << std::endl;
    
    return true;
}

bool BNO085::tareZAxis() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Taring Z-axis (heading only)..." << std::endl;
    std::cout << "Current heading will become the new zero heading." << std::endl;
    
    // Tare only Z axis (heading) using rotation vector as basis
    int result = sh2_setTareNow(SH2_TARE_Z, SH2_TARE_BASIS_ROTATION_VECTOR);
    
    if (result != SH2_OK) {
        setError("Failed to tare Z-axis: " + sh2ErrorToString(result));
        return false;
    }
    
    // Give sensor time to apply the tare
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Z-axis tared successfully." << std::endl;
    std::cout << "Note: For Game Rotation Vector, sensor must be reset for tare to take effect." << std::endl;
    std::cout << "Use persistTare() to save this tare to flash." << std::endl;
    
    return true;
}

bool BNO085::clearTare() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Clearing tare..." << std::endl;
    
    int result = sh2_clearTare();
    
    if (result != SH2_OK) {
        setError("Failed to clear tare: " + sh2ErrorToString(result));
        return false;
    }
    
    // Give sensor time to clear the tare
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Tare cleared. Sensor orientation reset to default." << std::endl;
    
    return true;
}

bool BNO085::persistTare() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Saving tare to flash memory..." << std::endl;
    
    int result = sh2_persistTare();
    
    if (result != SH2_OK) {
        setError("Failed to persist tare: " + sh2ErrorToString(result));
        return false;
    }
    
    // Give sensor time to complete the flash write
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Tare saved to flash. Will persist across resets." << std::endl;
    std::cout << "Note: Game Rotation Vector does not persist tare." << std::endl;
    
    return true;
}

bool BNO085::setCustomOrientation(const Quaternion& orientation) {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    std::cout << "Setting custom orientation: " << orientation.toString() << std::endl;
    
    // Convert our Quaternion to sh2_Quaternion_t
    sh2_Quaternion_t sh2_quat;
    sh2_quat.w = orientation.w;
    sh2_quat.x = orientation.x;
    sh2_quat.y = orientation.y;
    sh2_quat.z = orientation.z;

    int result = sh2_setReorientation(&sh2_quat);
    
    if (result != SH2_OK) {
        setError("Failed to set custom orientation: " + sh2ErrorToString(result));
        return false;
    }
    
    // Give sensor time to apply the reorientation
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Custom orientation applied." << std::endl;
    std::cout << "To clear this, call clearTare() or setCustomOrientation with identity quaternion." << std::endl;
    
    return true;
}

// =============================================================================
// DIAGNOSTICS AND STATUS
// =============================================================================

std::string BNO085::getProductInfo() {
    if (!initialized_) {
        return "Sensor not initialized";
    }
    
    sh2_ProductIds_t prodIds;
    int result = sh2_getProdIds(&prodIds);
    if (result != SH2_OK) {
        return "Failed to get product info: " + sh2ErrorToString(result);
    }
    
    std::ostringstream oss;
    oss << "BNO085 Product Information:\n";
    for (int i = 0; i < prodIds.numEntries; i++) {
        const auto& prod = prodIds.entry[i];
        oss << "  Entry " << i << ":\n";
        oss << "    Part Number: 0x" << std::hex << prod.swPartNumber << std::dec << "\n";
        oss << "    Version: " << static_cast<int>(prod.swVersionMajor) 
            << "." << static_cast<int>(prod.swVersionMinor) 
            << "." << prod.swVersionPatch << "\n";
        oss << "    Build: " << prod.swBuildNumber << "\n";
        oss << "    Reset Cause: " << static_cast<int>(prod.resetCause) << "\n";
    }
    
    return oss.str();
}

bool BNO085::isConnected() {
    if (!hal_) {
        return false;
    }
    return isI2CDevicePresent(hal_);
}

bool BNO085::reset() {
    if (!initialized_) {
        setError("Sensor not initialized");
        return false;
    }
    
    int result = sh2_devReset();
    if (result != SH2_OK) {
        setError("Failed to reset sensor: " + sh2ErrorToString(result));
        return false;
    }
    
    std::cout << "Sensor reset initiated" << std::endl;
    return true;
}

std::string BNO085::getSensorMetadata(sh2_SensorId_t sensor_id) {
    if (!initialized_) {
        return "Sensor not initialized";
    }
    
    sh2_SensorMetadata_t metadata;
    int result = sh2_getMetadata(sensor_id, &metadata);
    if (result != SH2_OK) {
        return "Failed to get metadata: " + sh2ErrorToString(result);
    }
    
    std::ostringstream oss;
    oss << "Sensor Metadata for " << sensorIdToString(sensor_id) << ":\n";
    oss << "  Range: " << metadata.range << "\n";
    oss << "  Resolution: " << metadata.resolution << "\n";
    oss << "  Power: " << (metadata.power_mA / 1024.0f) << " mA\n";
    oss << "  Min Period: " << metadata.minPeriod_uS << " µs\n";
    oss << "  Max Period: " << metadata.maxPeriod_uS << " µs\n";
    oss << "  Vendor: " << std::string(metadata.vendorId, metadata.vendorIdLen) << "\n";
    
    return oss.str();
}

bool BNO085::wasReset() {
    bool result = reset_occurred_;
    reset_occurred_ = false;  // Clear flag
    return result;
}

// =============================================================================
// PRIVATE METHODS
// =============================================================================

void BNO085::setError(const std::string& error) {
    last_error_ = error;
    std::cerr << "BNO085 Error: " << error << std::endl;
    
    if (error_callback_) {
        error_callback_(error);
    }
}

void BNO085::serviceLoop() {
    std::cout << "BNO085 service loop started" << std::endl;
    
    while (service_running_) {
        if (initialized_) {
            // Service the SH2 library - this processes incoming data
            sh2_service();
        }
        
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(1000));  // 1ms
    }
    
    std::cout << "BNO085 service loop ended" << std::endl;
}

// =============================================================================
// SH2 CALLBACK HANDLERS (STATIC)
// =============================================================================

void BNO085::eventCallbackWrapper(void* cookie, sh2_AsyncEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    if (instance) {
        instance->handleAsyncEvent(event);
    }
}

void BNO085::sensorCallbackWrapper(void* cookie, sh2_SensorEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    if (instance) {
        instance->handleSensorEvent(event);
    }
}

// =============================================================================
// INSTANCE CALLBACK HANDLERS
// =============================================================================

void BNO085::handleAsyncEvent(sh2_AsyncEvent_t* event) {
    switch (event->eventId) {
        case SH2_RESET:
            handleResetEvent();
            break;
            
        case SH2_GET_FEATURE_RESP:
            handleFeatureResponse(event->sh2SensorConfigResp);
            break;
            
        case SH2_SHTP_EVENT:
            handleShtpError(event->shtpEvent);
            break;
            
        default:
            std::cout << "BNO085: Unknown async event " << event->eventId << std::endl;
            break;
    }
}


void BNO085::handleFeatureResponse(const sh2_SensorConfigResp_t& resp) {
        std::cout << "Feature Response for " << sensorIdToString(resp.sensorId) << ":" << std::endl;
        std::cout << "  Report Interval: " << resp.sensorConfig.reportInterval_us << " µs "
                  << "(" << (1000000.0f / resp.sensorConfig.reportInterval_us) << " Hz)" << std::endl;
        
        if (resp.sensorConfig.batchInterval_us > 0) {
            std::cout << "  Batch Interval: " << resp.sensorConfig.batchInterval_us << " µs" << std::endl;
        }
        
        std::cout << "  Flags: ";
        if (resp.sensorConfig.changeSensitivityEnabled) {
            std::cout << "ChangeSensitivity("
                     << (resp.sensorConfig.changeSensitivityRelative ? "Relative" : "Absolute")
                     << "=" << resp.sensorConfig.changeSensitivity << ") ";
        }
        if (resp.sensorConfig.wakeupEnabled) {
            std::cout << "Wakeup ";
        }
        if (resp.sensorConfig.alwaysOnEnabled) {
            std::cout << "AlwaysOn ";
        }
        if (!resp.sensorConfig.changeSensitivityEnabled && 
            !resp.sensorConfig.wakeupEnabled && 
            !resp.sensorConfig.alwaysOnEnabled) {
            std::cout << "None";
        }
        std::cout << std::endl;
        
        if (resp.sensorConfig.sensorSpecific != 0) {
            std::cout << "  Sensor Specific: 0x" << std::hex << resp.sensorConfig.sensorSpecific 
                     << std::dec << std::endl;
        }
    }

void BNO085::handleShtpError(sh2_ShtpEvent_t shtpEvent) {
        std::string errorMsg = "SHTP Protocol Error: ";
        
        switch (shtpEvent) {
            case SH2_SHTP_TX_DISCARD:
                errorMsg += "TX Discard - Transmitted data was discarded";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: Communication buffer overflow" << std::endl;
                break;
                
            case SH2_SHTP_SHORT_FRAGMENT:
                errorMsg += "Short Fragment - Received fragment too short";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: I2C communication corruption" << std::endl;
                break;
                
            case SH2_SHTP_TOO_LARGE_PAYLOADS:
                errorMsg += "Too Large Payload - Payload exceeds buffer size";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: Sensor sending more data than expected" << std::endl;
                break;
                
            case SH2_SHTP_BAD_RX_CHAN:
                errorMsg += "Bad RX Channel - Invalid receive channel";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: Protocol desynchronization" << std::endl;
                break;
                
            case SH2_SHTP_BAD_TX_CHAN:
                errorMsg += "Bad TX Channel - Invalid transmit channel";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: Internal software error" << std::endl;
                break;
                
            case SH2_SHTP_BAD_FRAGMENT:
                errorMsg += "Bad Fragment - Malformed data fragment";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: I2C noise or timing issues" << std::endl;
                break;
                
            case SH2_SHTP_BAD_SN:
                errorMsg += "Bad Sequence Number - Data packet out of order";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: Missed packets or comm errors" << std::endl;
                break;
                
            case SH2_SHTP_INTERRUPTED_PAYLOAD:
                errorMsg += "Interrupted Payload - Data transfer was interrupted";
                std::cerr << errorMsg << std::endl;
                std::cerr << "  Possible cause: Reset during data transfer" << std::endl;
                break;
                
            default:
                errorMsg += "Unknown SHTP event code: " + std::to_string(shtpEvent);
                std::cerr << errorMsg << std::endl;
                break;
        }
    }


void BNO085::handleResetEvent() {
        std::cout << "=== BNO085 Reset Complete ===" << std::endl;
        
        // Set the reset flag
        reset_occurred_ = true;
        
        // Log reset information
        sh2_ProductIds_t prodIds;
        if (sh2_getProdIds(&prodIds) == SH2_OK && prodIds.numEntries > 0) {
            std::cout << "Reset cause: ";
            switch (prodIds.entry[0].resetCause) {
                case 0: std::cout << "Unknown" << std::endl; break;
                case 1: std::cout << "Power-On Reset" << std::endl; break;
                case 2: std::cout << "Internal System Reset" << std::endl; break;
                case 3: std::cout << "Watchdog Timer Reset" << std::endl; break;
                case 4: std::cout << "External Reset" << std::endl; break;
                case 5: std::cout << "Other" << std::endl; break;
                default: std::cout << "Code " << static_cast<int>(prodIds.entry[0].resetCause) << std::endl;
            }
        }
        
        std::cout << "All sensor configurations have been cleared." << std::endl;
        std::cout << "Re-enable sensors as needed." << std::endl;
        
        // Clear cached sensor data since it's no longer valid
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_acceleration_.clear();
            latest_angular_velocity_.clear();
            latest_magnetic_field_.clear();
            latest_orientation_.clear();
        }
        
        // Notify via error callback if registered (use as general event notification)
        if (error_callback_) {
            error_callback_("RESET_EVENT: Sensor hub has been reset");
        }


        // Auto-reconfigure sensors
    if (!enabled_sensors_.empty()) {
        std::cout << "Auto-reconfiguring " << enabled_sensors_.size() << " sensors..." << std::endl;
        
        for (auto it = enabled_sensors_.begin(); it != enabled_sensors_.end(); ++it) {
            sh2_SensorId_t sensor_id = it->first;
            const sh2_SensorConfig_t& config = it->second;
            
            std::cout << "  Re-enabling " << sensorIdToString(sensor_id) << "..." << std::endl;
            
            // Small delay between sensor configurations
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            int result = sh2_setSensorConfig(sensor_id, &config);
            if (result != SH2_OK) {
                std::cerr << "    Failed to re-enable: " << sh2ErrorToString(result) << std::endl;
            } else {
                std::cout << "    Success" << std::endl;
            }
        }
    }
}
    
void BNO085::handleSensorEvent(sh2_SensorEvent_t* event) {
    sh2_SensorValue_t value;
    
    // Decode the sensor event
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        setError("Failed to decode sensor event");
        return;
    }

    
    // Route to category-specific update methods
    switch (value.sensorId) {
        // Acceleration sensors
        case SH2_RAW_ACCELEROMETER:
        case SH2_ACCELEROMETER:
        case SH2_LINEAR_ACCELERATION:
        case SH2_GRAVITY:
            updateAccelerationData(value.sensorId, value);
            break;
            
        // Angular velocity sensors
        case SH2_RAW_GYROSCOPE:
        case SH2_GYROSCOPE_CALIBRATED:
        case SH2_GYROSCOPE_UNCALIBRATED:
            updateAngularVelocityData(value.sensorId, value);
            break;
            
        // Magnetic field sensors
        case SH2_RAW_MAGNETOMETER:
        case SH2_MAGNETIC_FIELD_CALIBRATED:
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            updateMagneticFieldData(value.sensorId, value);
            break;
            
        // Orientation sensors
        case SH2_ROTATION_VECTOR:
        case SH2_GAME_ROTATION_VECTOR:
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        case SH2_ARVR_STABILIZED_RV:
        case SH2_ARVR_STABILIZED_GRV:
        case SH2_GYRO_INTEGRATED_RV:
            updateOrientationData(value.sensorId, value);
            break;
            
        default:
            // Other sensors not yet categorized
            break;
    }
    
    // Call generic sensor callback if registered
    if (sensor_callback_) {
        sensor_callback_(value.sensorId, value);
    }
}

// =============================================================================
// CATEGORY-SPECIFIC UPDATE METHODS
// =============================================================================

void BNO085::updateAccelerationData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value) {
    AccelerationReading reading;
    reading.meta.sensor_id = sensor_id;
    reading.meta.sequence = value.sequence;
    reading.meta.status = StatustoString(value.status);
    reading.meta.delay = value.delay;
    reading.meta.report_timestamp = value.timestamp;

    // Extract acceleration based on sensor type
    switch (sensor_id) {
        case SH2_RAW_ACCELEROMETER:
            reading.acceleration.x = value.un.rawAccelerometer.x;
            reading.acceleration.y = value.un.rawAccelerometer.y;
            reading.acceleration.z = value.un.rawAccelerometer.z;
            reading.sensor_timestamp = value.un.rawAccelerometer.timestamp;
            reading.has_timestamp = true;
            break;
            
        case SH2_ACCELEROMETER:
            reading.acceleration.x = value.un.accelerometer.x;
            reading.acceleration.y = value.un.accelerometer.y;
            reading.acceleration.z = value.un.accelerometer.z;
            reading.has_timestamp = false;
            break;
            
        case SH2_LINEAR_ACCELERATION:
            reading.acceleration.x = value.un.linearAcceleration.x;
            reading.acceleration.y = value.un.linearAcceleration.y;
            reading.acceleration.z = value.un.linearAcceleration.z;
            reading.has_timestamp = false;
            break;
            
        case SH2_GRAVITY:
            reading.acceleration.x = value.un.gravity.x;
            reading.acceleration.y = value.un.gravity.y;
            reading.acceleration.z = value.un.gravity.z;
            reading.has_timestamp = false;
            break;
            
        default:
            return;
    }
    
    // Store in map
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_acceleration_[sensor_id] = reading;
    }
    
    // Trigger sensor-specific callback if registered
    auto it = accel_callbacks_.find(sensor_id);
    if (it != accel_callbacks_.end() && it->second) {
        it->second(reading);
    }
    
    // Trigger category-wide callback if registered
    if (accel_callback_) {
        accel_callback_(reading);
    }
}

void BNO085::updateAngularVelocityData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value) {
    AngularVelocityReading reading;
    reading.meta.sensor_id = sensor_id;
    reading.meta.sequence = value.sequence;
    reading.meta.status = StatustoString(value.status);
    reading.meta.delay = value.delay;
    reading.meta.report_timestamp = value.timestamp;

    // Extract angular velocity based on sensor type
    switch (sensor_id) {
        case SH2_RAW_GYROSCOPE:
            reading.angular_velocity.x = value.un.rawGyroscope.x;
            reading.angular_velocity.y = value.un.rawGyroscope.y;
            reading.angular_velocity.z = value.un.rawGyroscope.z;
            reading.sensor_timestamp = value.un.rawGyroscope.timestamp;
            reading.has_timestamp = true;
            reading.has_bias = false;
            break;
            
        case SH2_GYROSCOPE_CALIBRATED:
            reading.angular_velocity.x = value.un.gyroscope.x;
            reading.angular_velocity.y = value.un.gyroscope.y;
            reading.angular_velocity.z = value.un.gyroscope.z;
            reading.has_timestamp = false;
            reading.has_bias = false;
            break;
            
        case SH2_GYROSCOPE_UNCALIBRATED:
            reading.angular_velocity.x = value.un.gyroscopeUncal.x;
            reading.angular_velocity.y = value.un.gyroscopeUncal.y;
            reading.angular_velocity.z = value.un.gyroscopeUncal.z;
            reading.bias.x = value.un.gyroscopeUncal.biasX;
            reading.bias.y = value.un.gyroscopeUncal.biasY;
            reading.bias.z = value.un.gyroscopeUncal.biasZ;
            reading.has_bias = true;
            break;
            
        default:
            return;
    }
    
    // Store in map
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_angular_velocity_[sensor_id] = reading;
    }
    
    // Trigger sensor-specific callback if registered
    auto it = angular_vel_callbacks_.find(sensor_id);
    if (it != angular_vel_callbacks_.end() && it->second) {
        it->second(reading);
    }
    
    // Trigger category-wide callback if registered
    if (angular_vel_callback_) {
        angular_vel_callback_(reading);
    }
}

void BNO085::updateMagneticFieldData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value) {
    MagneticFieldReading reading;
    reading.meta.sensor_id = sensor_id;
    reading.meta.sequence = value.sequence;
    reading.meta.status = StatustoString(value.status);
    reading.meta.delay = value.delay;
    reading.meta.report_timestamp = value.timestamp;

    // Extract magnetic field based on sensor type
    switch (sensor_id) {
        case SH2_RAW_MAGNETOMETER:
            reading.magnetic_field.x = value.un.rawMagnetometer.x;
            reading.magnetic_field.y = value.un.rawMagnetometer.y;
            reading.magnetic_field.z = value.un.rawMagnetometer.z;
            reading.has_bias = false;
            break;
            
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            reading.magnetic_field.x = value.un.magneticField.x;
            reading.magnetic_field.y = value.un.magneticField.y;
            reading.magnetic_field.z = value.un.magneticField.z;
            reading.has_bias = false;
            break;
            
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            reading.magnetic_field.x = value.un.magneticFieldUncal.x;
            reading.magnetic_field.y = value.un.magneticFieldUncal.y;
            reading.magnetic_field.z = value.un.magneticFieldUncal.z;
            reading.bias.x = value.un.magneticFieldUncal.biasX;
            reading.bias.y = value.un.magneticFieldUncal.biasY;
            reading.bias.z = value.un.magneticFieldUncal.biasZ;
            reading.has_bias = true;
            break;
            
        default:
            return;
    }
    
    // Store in map
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_magnetic_field_[sensor_id] = reading;
    }
    
    // Trigger sensor-specific callback if registered
    auto it = mag_field_callbacks_.find(sensor_id);
    if (it != mag_field_callbacks_.end() && it->second) {
        it->second(reading);
    }
    
    // Trigger category-wide callback if registered
    if (mag_field_callback_) {
        mag_field_callback_(reading);
    }
}

void BNO085::updateOrientationData(sh2_SensorId_t sensor_id, const sh2_SensorValue_t& value) {
    OrientationReading reading;
    reading.meta.sensor_id = sensor_id;
    reading.meta.sequence = value.sequence;
    reading.meta.status = StatustoString(value.status);
    reading.meta.delay = value.delay;
    reading.meta.report_timestamp = value.timestamp;

    // Extract quaternion based on sensor type
    switch (sensor_id) {
        case SH2_ROTATION_VECTOR:
            reading.rotation.w = value.un.rotationVector.real;
            reading.rotation.x = value.un.rotationVector.i;
            reading.rotation.y = value.un.rotationVector.j;
            reading.rotation.z = value.un.rotationVector.k;
            reading.rotation.accuracy = value.un.rotationVector.accuracy;
            reading.rotation.has_accuracy = true;
            reading.has_angular_velocity = false;

            break;
            
        case SH2_GAME_ROTATION_VECTOR:
            reading.rotation.w = value.un.gameRotationVector.real;
            reading.rotation.x = value.un.gameRotationVector.i;
            reading.rotation.y = value.un.gameRotationVector.j;
            reading.rotation.z = value.un.gameRotationVector.k;
            reading.rotation.has_accuracy = false;
            reading.has_angular_velocity = false;

            break;
            
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            reading.rotation.w = value.un.geoMagRotationVector.real;
            reading.rotation.x = value.un.geoMagRotationVector.i;
            reading.rotation.y = value.un.geoMagRotationVector.j;
            reading.rotation.z = value.un.geoMagRotationVector.k;
            reading.rotation.accuracy = value.un.geoMagRotationVector.accuracy;
            reading.rotation.has_accuracy = true;
            reading.has_angular_velocity = false;

            break;
            
        case SH2_ARVR_STABILIZED_RV:
            reading.rotation.w = value.un.arvrStabilizedRV.real;
            reading.rotation.x = value.un.arvrStabilizedRV.i;
            reading.rotation.y = value.un.arvrStabilizedRV.j;
            reading.rotation.z = value.un.arvrStabilizedRV.k;
            reading.rotation.accuracy = value.un.arvrStabilizedRV.accuracy;
            reading.rotation.has_accuracy = true;
            reading.has_angular_velocity = false;

            break;
            
        case SH2_ARVR_STABILIZED_GRV:
            reading.rotation.w = value.un.arvrStabilizedGRV.real;
            reading.rotation.x = value.un.arvrStabilizedGRV.i;
            reading.rotation.y = value.un.arvrStabilizedGRV.j;
            reading.rotation.z = value.un.arvrStabilizedGRV.k;
            reading.rotation.has_accuracy = false;
            reading.has_angular_velocity = false;
            break;
            
        case SH2_GYRO_INTEGRATED_RV:
            reading.rotation.w = value.un.gyroIntegratedRV.real;
            reading.rotation.x = value.un.gyroIntegratedRV.i;
            reading.rotation.y = value.un.gyroIntegratedRV.j;
            reading.rotation.z = value.un.gyroIntegratedRV.k;
            reading.angular_velocity.x = value.un.gyroIntegratedRV.angVelX;
            reading.angular_velocity.y = value.un.gyroIntegratedRV.angVelY;
            reading.angular_velocity.z = value.un.gyroIntegratedRV.angVelZ;
            reading.rotation.has_accuracy = false;
            reading.has_angular_velocity = true;
            break;
            
        default:
            return;
    }
    
    // Compute euler angles
    reading.euler = reading.rotation.toEulerAngles();
    
    // Store in map
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_orientation_[sensor_id] = reading;
    }
    
    // Trigger sensor-specific callback if registered
    auto it = orientation_callbacks_.find(sensor_id);
    if (it != orientation_callbacks_.end() && it->second) {
        it->second(reading);
    }
    
    // Trigger category-wide callback if registered
    if (orientation_callback_) {
        orientation_callback_(reading);
    }
}
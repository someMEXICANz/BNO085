

//////////////////////////////////////////
//// Basic Main Example               ////
//////////////////////////////////////////

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

    sh2_SensorId_t Accel = sh2_SensorId_e::SH2_ACCELEROMETER;
    sh2_SensorId_t Gyro = sh2_SensorId_e::SH2_GYROSCOPE_CALIBRATED;
    sh2_SensorId_t Mag = sh2_SensorId_e::SH2_MAGNETIC_FIELD_CALIBRATED;
    sh2_SensorId_t Orient = sh2_SensorId_e::SH2_GAME_ROTATION_VECTOR;


    sensor.enableSensor(Accel, 100.0f);
    sensor.enableSensor(Gyro, 100.0f);
    sensor.enableSensor(Mag, 50.0f);
    sensor.enableSensor(Orient, 50.0f);

    BNO085::AccelerationReading Accel_reading;
    BNO085::AngularVelocityReading AV_reading;
    BNO085::MagneticFieldReading Mag_reading;
    BNO085::OrientationReading Orient_reading;

    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (true) {

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        sensor.getAccelerationData(Accel, Accel_reading);
        std::cout << Accel_reading.meta.printMetaData();
        std::cout << sensorIdToString(Accel) << ": "
                  << Accel_reading.acceleration.toString() << std::endl;
        std::cout << "////////////////////////////////////////// " << std::endl;

        sensor.getAngularVelocityData(Gyro, AV_reading);
        std::cout << AV_reading.meta.printMetaData();
        std::cout << sensorIdToString(Gyro) << ": "
                  << AV_reading.angular_velocity.toString() << std::endl;
        std::cout << "////////////////////////////////////////// " << std::endl;

        sensor.getMagneticFieldData(Mag, Mag_reading);
        std::cout << Mag_reading.meta.printMetaData();
        std::cout << sensorIdToString(Mag) << ": "
                  << Mag_reading.magnetic_field.toString() << std::endl;
        std::cout << "////////////////////////////////////////// " << std::endl;

        sensor.getOrientationData(Orient, Orient_reading);
        std::cout << Orient_reading.meta.printMetaData();
        std::cout << sensorIdToString(Orient) << ": "
                  << Orient_reading.rotation.toString() << std::endl;
        std::cout << "////////////////////////////////////////// " << std::endl;
       
      
    }
    
    return 0;
}


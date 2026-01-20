// sh2_c_interface.h
#ifndef SH2_C_INTERFACE_H
#define SH2_C_INTERFACE_H

/**
 * @file sh2_c_interface.h
 * @brief C/C++ interface bridge for SH2 sensor hub library
 * 
 * This header provides a clean way to include the SH2 C library
 * in C++ code without having to write extern "C" everywhere.
 * 
 * Simply include this header instead of the individual SH2 headers.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Core SH2 library headers
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_util.h"
#include "euler.h"
#include "shtp.h"

#ifdef __cplusplus
}
#endif

// C++ convenience additions
#ifdef __cplusplus

#include <string>
#include <iostream>
#include <math.h> 


/**
 * @brief Convert SH2 error code to string
 * @param error SH2 error code
 * @return Human-readable error description
 */
inline std::string sh2ErrorToString(int error) {
    switch (error) {
        case SH2_OK:                 return "Success";
        case SH2_ERR:                return "General Error";
        case SH2_ERR_BAD_PARAM:      return "Bad Parameter";
        case SH2_ERR_OP_IN_PROGRESS: return "Operation In Progress";
        case SH2_ERR_IO:             return "I/O Error";
        case SH2_ERR_HUB:            return "Hub Error";
        case SH2_ERR_TIMEOUT:        return "Timeout";
        default:                     return "Unknown Error (" + std::to_string(error) + ")";
    }
}

/**
 * @brief Convert sensor ID to string
 * @param sensorId SH2 sensor ID
 * @return Human-readable sensor name
 */
inline std::string sensorIdToString(sh2_SensorId_t sensorId) {
    switch (sensorId) {
        case SH2_RAW_ACCELEROMETER:           return "Raw Accelerometer";
        case SH2_ACCELEROMETER:               return "Accelerometer";
        case SH2_LINEAR_ACCELERATION:         return "Linear Acceleration";
        case SH2_GRAVITY:                     return "Gravity";
        case SH2_RAW_GYROSCOPE:               return "Raw Gyroscope";
        case SH2_GYROSCOPE_CALIBRATED:        return "Gyroscope (Calibrated)";
        case SH2_GYROSCOPE_UNCALIBRATED:      return "Gyroscope (Uncalibrated)";
        case SH2_RAW_MAGNETOMETER:            return "Raw Magnetometer";
        case SH2_MAGNETIC_FIELD_CALIBRATED:   return "Magnetic Field (Calibrated)";
        case SH2_MAGNETIC_FIELD_UNCALIBRATED: return "Magnetic Field (Uncalibrated)";
        case SH2_ROTATION_VECTOR:             return "Rotation Vector";
        case SH2_GAME_ROTATION_VECTOR:        return "Game Rotation Vector";
        case SH2_GEOMAGNETIC_ROTATION_VECTOR: return "Geomagnetic Rotation Vector";
        case SH2_PRESSURE:                    return "Pressure";
        case SH2_AMBIENT_LIGHT:               return "Ambient Light";
        case SH2_HUMIDITY:                    return "Humidity";
        case SH2_PROXIMITY:                   return "Proximity";
        case SH2_TEMPERATURE:                 return "Temperature";
        case SH2_TAP_DETECTOR:                return "Tap Detector";
        case SH2_STEP_DETECTOR:               return "Step Detector";
        case SH2_STEP_COUNTER:                return "Step Counter";
        case SH2_SIGNIFICANT_MOTION:          return "Significant Motion";
        case SH2_STABILITY_CLASSIFIER:        return "Stability Classifier";
        case SH2_SHAKE_DETECTOR:              return "Shake Detector";
        case SH2_FLIP_DETECTOR:               return "Flip Detector";
        case SH2_PICKUP_DETECTOR:             return "Pickup Detector";
        case SH2_STABILITY_DETECTOR:          return "Stability Detector";
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER: return "Personal Activity Classifier";
        case SH2_SLEEP_DETECTOR:              return "Sleep Detector";
        case SH2_TILT_DETECTOR:               return "Tilt Detector";
        case SH2_POCKET_DETECTOR:             return "Pocket Detector";
        case SH2_CIRCLE_DETECTOR:             return "Circle Detector";
        case SH2_HEART_RATE_MONITOR:          return "Heart Rate Monitor";
        case SH2_ARVR_STABILIZED_RV:          return "AR/VR Stabilized RV";
        case SH2_ARVR_STABILIZED_GRV:         return "AR/VR Stabilized GRV";
        case SH2_GYRO_INTEGRATED_RV:          return "Gyro Integrated RV";
        case SH2_IZRO_MOTION_REQUEST:         return "iZRO Motion Request";
        case SH2_RAW_OPTICAL_FLOW:            return "Raw Optical Flow";
        case SH2_DEAD_RECKONING_POSE:         return "Dead Reckoning Pose";
        case SH2_WHEEL_ENCODER:               return "Wheel Encoder";
        default:                              return "Unknown Sensor (0x" + 
                                                     std::to_string(sensorId) + ")";
    }
}

/**
 * @brief Convert SH2 status code to string
 * @param sh2_status SH2 status code
 * @return Human-readable status description
 */
inline std::string StatustoString(uint8_t sh2_status) {
    switch (sh2_status) {
        case 0: return "UNRELIABLE";
        case 1: return "LOW";
        case 2: return "MEDIUM";
        case 3: return "HIGH";
        default: return "UNRELIABLE"; // Fallback for unknown values
    }
}

/**
 * @brief Convert calibration status to string
 * @param status SH2 calibration status
 * @return Human-readable calibration status description
 */
inline std::string calibrationStatusToString(sh2_CalStatus_t status) {
    switch (status) {
        case sh2_CalStatus_t::SH2_CAL_SUCCESS:                  return "Success";
        case sh2_CalStatus_t::SH2_CAL_NO_ZRO:                   return "No zero-rate output detected";
        case sh2_CalStatus_t::SH2_CAL_NO_STATIONARY_DETECTION:  return "No stationary period detected";
        case sh2_CalStatus_t::SH2_CAL_ROTATION_OUTSIDE_SPEC:    return "Rotation outside specification";
        case sh2_CalStatus_t::SH2_CAL_ZRO_OUTSIDE_SPEC:         return "Zero-rate output outside specification";
        case sh2_CalStatus_t::SH2_CAL_ZGO_OUTSIDE_SPEC:         return "Zero-g output outside specification";
        case sh2_CalStatus_t::SH2_CAL_GYRO_GAIN_OUTSIDE_SPEC:   return "Gyro gain outside specification";
        case sh2_CalStatus_t::SH2_CAL_GYRO_PERIOD_OUTSIDE_SPEC: return "Gyro period outside specification";
        case sh2_CalStatus_t::SH2_CAL_GYRO_DROPS_OUTSIDE_SPEC:  return "Gyro drops outside specification";
        default:                                                return "Unknown status (" + 
                                                                        std::to_string(static_cast<int>(status)) + ")";
    }
}

#endif // __cplusplus

#endif // SH2_C_INTERFACE_H
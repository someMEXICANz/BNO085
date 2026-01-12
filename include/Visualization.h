// Visualization.h
#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "BNO085.h"
#include <string>
#include <atomic>
#include <mutex>
#include <memory>
#include <open3d/Open3D.h>
#include <Eigen/Dense>

// =============================================================================
// GLOBAL STATE
// =============================================================================

struct SensorState {
    std::mutex mutex;
    BNO085::Quaternion orientation;
    BNO085::Vector3 acceleration;
    bool data_available;
    std::atomic<bool> running;
    open3d::visualization::Visualizer visualizer;
    
    SensorState() : data_available(false), running(true) {}
};

extern SensorState g_sensor_state;

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

Eigen::Quaterniond bnoQuatToEigen(const BNO085::Quaternion& q);

Eigen::Matrix3d quaternionToRotationMatrix(const BNO085::Quaternion& q);

std::shared_ptr<open3d::geometry::TriangleMesh> loadPLYModel(const std::string& filename);

std::shared_ptr<open3d::geometry::TriangleMesh> createFallbackCube();

std::shared_ptr<open3d::geometry::TriangleMesh> createCoordinateFrame(double size = 0.3);

void printSensorData(const BNO085::Quaternion& q, const BNO085::Vector3& accel);

// =============================================================================
// CALLBACKS
// =============================================================================

void orientationCallback(const BNO085::OrientationData& data);

void imuCallback(const BNO085::IMUData& data);

// =============================================================================
// VISUALIZATION THREAD
// =============================================================================

void visualizationThread(const std::string& ply_file);

int startVisualization(BNO085& sensor, const std::string& model_path);

// =============================================================================
// SIGNAL HANDLER
// =============================================================================

void signalHandler(int signal);

#endif // VISUALIZATION_H
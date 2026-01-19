
// #include "BNO085.h"
// #include <iostream>
// #include <iomanip>
// #include <signal.h>
// #include <thread>
// #include <chrono>
// #include <atomic>
// #include <mutex>
// #include <memory>

// // Open3D headers
// #include <open3d/Open3D.h>
// #include <Eigen/Dense>


// // =============================================================================
// // GLOBAL STATE
// // =============================================================================

// struct SensorState {
//     std::mutex mutex;
//     BNO085::Quaternion orientation;
//     BNO085::Vector3 acceleration;
//     bool data_available = false;
//     std::atomic<bool> running{true};
//     open3d::visualization::Visualizer visualizer;
// };

// SensorState g_sensor_state;

// // =============================================================================
// // HELPER FUNCTIONS
// // =============================================================================

// Eigen::Quaterniond bnoQuatToEigen(const BNO085::Quaternion& q) {
//     return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
// }

// Eigen::Matrix3d quaternionToRotationMatrix(const BNO085::Quaternion& q) {
//     Eigen::Quaterniond eigen_quat = bnoQuatToEigen(q);
//     return eigen_quat.normalized().toRotationMatrix();
// }

// std::shared_ptr<open3d::geometry::TriangleMesh> loadPLYModel(const std::string& filename) {
//     auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    
//     if (!open3d::io::ReadTriangleMesh(filename, *mesh)) {
//         std::cerr << "Failed to load: " << filename << std::endl;
//         return nullptr;
//     }
    
//     std::cout << "Loaded mesh with " << mesh->vertices_.size() << " vertices" << std::endl;
    
//     mesh->Translate(-mesh->GetCenter());
//     double scale = mesh->GetMaxBound().norm();
//     if (scale > 0) mesh->Scale(1.0 / scale, mesh->GetCenter());
//     mesh->ComputeVertexNormals();
    
//     return mesh;
// }

// std::shared_ptr<open3d::geometry::TriangleMesh> createFallbackCube() {
//     auto mesh = open3d::geometry::TriangleMesh::CreateBox(1.0, 1.0, 1.0);
//     mesh->Translate(-mesh->GetCenter());
//     mesh->ComputeVertexNormals();
//     mesh->PaintUniformColor(Eigen::Vector3d(0.7, 0.7, 0.9));
//     std::cout << "Using fallback cube" << std::endl;
//     return mesh;
// }

// std::shared_ptr<open3d::geometry::TriangleMesh> createCoordinateFrame(double size = 0.3) {
//     return open3d::geometry::TriangleMesh::CreateCoordinateFrame(size);
// }

// void printSensorData(const BNO085::Quaternion& q, const BNO085::Vector3& accel) {
//     std::cout << "\r";
//     std::cout << "Q: " << std::fixed << std::setprecision(3) 
//               << "w=" << q.w << " x=" << q.x << " y=" << q.y << " z=" << q.z;
    
//     BNO085::Vector3 euler = q.toEulerAngles();
//     std::cout << " | Euler: "
//               << "Y=" << std::setw(6) << euler.x << "° "
//               << "P=" << std::setw(6) << euler.y << "° "
//               << "R=" << std::setw(6) << euler.z << "°";
    
//     std::cout << " | Acc: " << accel.toString();
//     std::cout << std::flush;
// }

// // =============================================================================
// // CALLBACKS
// // =============================================================================

// void orientationCallback(const BNO085::OrientationData& data) {
//     std::lock_guard<std::mutex> lock(g_sensor_state.mutex);
//     g_sensor_state.orientation = data.rotation;
//     g_sensor_state.data_available = true;
// }

// void imuCallback(const BNO085::IMUData& data) {
//     std::lock_guard<std::mutex> lock(g_sensor_state.mutex);
//     g_sensor_state.acceleration = data.acceleration;
// }

// // =============================================================================
// // VISUALIZATION THREAD
// // =============================================================================

// void visualizationThread(const std::string& ply_file) {
//     std::cout << "\n=== Starting Visualization ===" << std::endl;
    
//     // Load model
//     std::shared_ptr<open3d::geometry::TriangleMesh> mesh = loadPLYModel(ply_file);
//     if (!mesh) mesh = createFallbackCube();
    
//     auto coord_frame = createCoordinateFrame(0.3);
    
//     // Create visualizer
//     open3d::visualization::Visualizer visualizer;
//     visualizer.CreateVisualizerWindow("BNO085 IMU - 3D Orientation", 1280, 720);
//     visualizer.AddGeometry(mesh);
//     visualizer.AddGeometry(coord_frame);
    
//     // Configure rendering
//     auto& render_option = visualizer.GetRenderOption();
//     render_option.background_color_ = Eigen::Vector3d(0.1, 0.1, 0.15);
//     render_option.mesh_show_back_face_ = true;
//     render_option.light_on_ = true;
    
//     auto& view_control = visualizer.GetViewControl();
//     view_control.SetZoom(0.6);
    
//     std::cout << "Visualization ready. Move your sensor!\n" << std::endl;
    
//     // Store original vertices
//     std::vector<Eigen::Vector3d> original_vertices = mesh->vertices_;
    
//     // Main loop
//     while (g_sensor_state.running && visualizer.PollEvents()) {
//         BNO085::Quaternion quat;
//         BNO085::Vector3 accel;
//         bool has_data = false;
        
//         {
//             std::lock_guard<std::mutex> lock(g_sensor_state.mutex);
//             if (g_sensor_state.data_available) {
//                 quat = g_sensor_state.orientation;
//                 accel = g_sensor_state.acceleration;
//                 has_data = true;
//             }
//         }
        
//         if (has_data) {
//             Eigen::Matrix3d rotation = quaternionToRotationMatrix(quat);
//             mesh->vertices_ = original_vertices;
//             mesh->Rotate(rotation, mesh->GetCenter());
//             mesh->ComputeVertexNormals();
//             printSensorData(quat, accel);
//         }
        
//         visualizer.UpdateGeometry(mesh);
//         visualizer.UpdateRender();
//         std::this_thread::sleep_for(std::chrono::milliseconds(16));
//     }
    
//     std::cout << "\n\nClosing visualization..." << std::endl;
//     visualizer.DestroyVisualizerWindow();
//     g_sensor_state.running = false;
// }

// // =============================================================================
// // SIGNAL HANDLER
// // =============================================================================

// // void signalHandler(int signal) {
// //     std::cout << "\n\nShutting down..." << std::endl;
// //     g_sensor_state.running = false;
// // }



// int startVisualization(BNO085& sensor, const std::string& model_path)
// {
//     sensor.setOrientationCallback(orientationCallback);
//     sensor.setIMUCallback(imuCallback);
    
//     if (!sensor.enableOrientation()) {
//         std::cerr << "Failed to enable orientation" << std::endl;
//         return 1;
//     }
    
//     if (!sensor.enableBasicIMU()) {
//         std::cerr << "Failed to enable IMU" << std::endl;
//         return 1;
//     }
    
//     sensor.startService();
    
//     // Wait for data
//     std::cout << "Waiting for sensor data..." << std::endl;
//     int wait_count = 0;
//     while (!g_sensor_state.data_available && wait_count < 50) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         wait_count++;
//     }
    
//     if (g_sensor_state.data_available) {
//         std::cout << "Sensor ready!" << std::endl;
//     }
    
//     // std::cout << "\n" << sensor.getProductInfo() << std::endl;
//     visualizationThread(model_path);
//     // Start visualization
//     std::thread vis_thread(visualizationThread, model_path);
    
//     while (g_sensor_state.running) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
    
//     if (vis_thread.joinable()) vis_thread.join();

//     return 0;

// }


// GUI.h - Updated
#ifndef GUI_H
#define GUI_H

#include "BNO085.h"
#include <open3d/Open3D.h>
#include <memory>
#include <string>
#include <mutex>

class IMUVisualizer {
public:
    IMUVisualizer(const std::string& model_path);
    ~IMUVisualizer();
    
    void Run();
    void UpdateSensorData(const BNO085::Quaternion& quat, 
                          const BNO085::Vector3& accel);
    
    bool IsRunning() const { return is_running_; }
    
private:
    void LoadModel(const std::string& path);
    void InitializeScene();
    
    std::shared_ptr<open3d::visualization::visualizer::O3DVisualizer> vis_;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
    std::shared_ptr<open3d::geometry::TriangleMesh> coord_frame_;
    std::vector<Eigen::Vector3d> original_vertices_;
    
    std::mutex data_mutex_;
    BNO085::Quaternion current_quat_;
    BNO085::Vector3 current_accel_;
    
    std::atomic<bool> is_running_;
    std::atomic<bool> scene_initialized_;
    bool data_available_;
};

#endif // GUI_H
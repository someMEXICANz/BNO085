// GUI.h - Enhanced with Interactive Controls
#ifndef GUI_H
#define GUI_H

#include "BNO085.h"
#include <open3d/Open3D.h>
#include <memory>
#include <string>
#include <mutex>
#include <map>

class IMUVisualizer {
public:
    IMUVisualizer(const std::string& model_path, BNO085& sensor);
    ~IMUVisualizer();
    
    void Run();
    void UpdateSensorData(const BNO085::Quaternion& quat, 
                          const BNO085::Vector3& accel);
    
    bool IsRunning() const { return is_running_; }
    
private:
    // Initialization
    void LoadModel(const std::string& path);
    void InitializeScene();
    void SetupUI();
    
    // UI Callbacks
    void OnSensorTypeChanged(const char* sensor_name, int index);
    void OnSamplingRateChanged(const char* label, double value);
    void OnEnableSensorToggled(bool enabled);
    void OnCalibrateClicked();
    void OnSaveCalibrationClicked();
    void OnResetSensorClicked();
    
    // UI Update functions
    void UpdateCalibrationStatus();
    void UpdateSensorInfo();
    void RefreshSamplingRateDropdown();
    
    // Helper functions
    BNO085::SensorType GetCurrentSensorType() const;
    void ApplySensorConfiguration();
    
    // 3D Visualization
    std::shared_ptr<open3d::visualization::visualizer::O3DVisualizer> vis_;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
    std::shared_ptr<open3d::geometry::TriangleMesh> coord_frame_;
    std::vector<Eigen::Vector3d> original_vertices_;
    
    // Sensor reference
    BNO085& sensor_;
    
    // UI Widgets
    std::shared_ptr<open3d::visualization::gui::Vert> settings_panel_;
    std::shared_ptr<open3d::visualization::gui::Combobox> sensor_type_dropdown_;
    std::shared_ptr<open3d::visualization::gui::Combobox> sampling_rate_dropdown_;
    std::shared_ptr<open3d::visualization::gui::Checkbox> enable_sensor_checkbox_;
    std::shared_ptr<open3d::visualization::gui::Label> calibration_label_;
    std::shared_ptr<open3d::visualization::gui::Label> sensor_info_label_;
    std::shared_ptr<open3d::visualization::gui::Button> calibrate_button_;
    std::shared_ptr<open3d::visualization::gui::Button> save_cal_button_;
    std::shared_ptr<open3d::visualization::gui::Button> reset_button_;
    std::shared_ptr<open3d::visualization::gui::TabControl> data_tabs_;

    // Tab 1: Current Sensor Data
    std::shared_ptr<open3d::visualization::gui::Label> accel_label_;
    std::shared_ptr<open3d::visualization::gui::Label> gyro_label_;
    std::shared_ptr<open3d::visualization::gui::Label> mag_label_;

    // Tab 2: Euler Angles
    std::shared_ptr<open3d::visualization::gui::Label> yaw_label_;
    std::shared_ptr<open3d::visualization::gui::Label> pitch_label_;
    std::shared_ptr<open3d::visualization::gui::Label> roll_label_;

    // Tab 3: Quaternion
    std::shared_ptr<open3d::visualization::gui::Label> quat_w_label_;
    std::shared_ptr<open3d::visualization::gui::Label> quat_x_label_;
    std::shared_ptr<open3d::visualization::gui::Label> quat_y_label_;
    std::shared_ptr<open3d::visualization::gui::Label> quat_z_label_;
    std::shared_ptr<open3d::visualization::gui::Label> quat_accuracy_label_;

    
    // Data synchronization
    std::mutex data_mutex_;
    BNO085::Quaternion current_quat_;
    BNO085::Vector3 current_accel_;
    
    // State
    std::atomic<bool> is_running_;
    std::atomic<bool> scene_initialized_;
    bool data_available_;
    
    // Configuration state
    int current_sensor_index_;
    std::map<BNO085::SensorType, BNO085::SensorConfig> sensor_configs_;
    
    // Sensor type mappings
    static const std::vector<std::pair<std::string, BNO085::SensorType>> sensor_types_;
    static const std::vector<std::pair<std::string, float>> sampling_rates_;
};

#endif // GUI_H
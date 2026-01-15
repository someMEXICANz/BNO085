// GUI.cpp - Enhanced with Interactive Controls
#include "GUI.h"
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace open3d;
using namespace open3d::visualization;

// Static member definitions
const std::vector<std::pair<std::string, BNO085::SensorType>> IMUVisualizer::sensor_types_ = {
    {"Rotation Vector", BNO085::SensorType::ROTATION_VECTOR},
    {"Game Rotation Vector", BNO085::SensorType::GAME_ROTATION_VECTOR},
    {"Accelerometer", BNO085::SensorType::ACCELEROMETER},
    {"Gyroscope", BNO085::SensorType::GYROSCOPE},
    {"Magnetometer", BNO085::SensorType::MAGNETOMETER},
    {"Linear Acceleration", BNO085::SensorType::LINEAR_ACCELERATION},
    {"Gravity", BNO085::SensorType::GRAVITY}
};

const std::vector<std::pair<std::string, float>> IMUVisualizer::sampling_rates_ = {
    {"1 Hz", 1.0f},
    {"5 Hz", 5.0f},
    {"10 Hz", 10.0f},
    {"20 Hz", 20.0f},
    {"50 Hz", 50.0f},
    {"100 Hz", 100.0f},
    {"200 Hz", 200.0f},
    {"400 Hz", 400.0f}
};

IMUVisualizer::IMUVisualizer(const std::string& model_path, BNO085& sensor)
    : sensor_(sensor),
      is_running_(false), 
      scene_initialized_(false), 
      data_available_(false),
      current_sensor_index_(0) {
    
    current_quat_ = BNO085::Quaternion(1, 0, 0, 0);
    current_accel_ = BNO085::Vector3(0, 0, 0);
    
    LoadModel(model_path);
    
    // Initialize default sensor configurations
    for (const auto& sensor_pair : sensor_types_) {
        BNO085::SensorConfig config;
        config.setFrequency(100.0f);  // Default 100 Hz
        config.enabled = false;
        sensor_configs_[sensor_pair.second] = config;
    }
}

IMUVisualizer::~IMUVisualizer() {
    is_running_ = false;
}

void IMUVisualizer::LoadModel(const std::string& path) {
    mesh_ = std::make_shared<geometry::TriangleMesh>();
    
    if (io::ReadTriangleMesh(path, *mesh_)) {
        std::cout << "Loaded mesh: " << path << std::endl;
        mesh_->Translate(-mesh_->GetCenter());
        double scale = mesh_->GetMaxBound().norm();
        if (scale > 0) mesh_->Scale(1.0 / scale, mesh_->GetCenter());
        mesh_->ComputeVertexNormals();
    } else {
        std::cerr << "Failed to load model, using cube" << std::endl;
        mesh_ = geometry::TriangleMesh::CreateBox(1.0, 1.0, 1.0);
        mesh_->Translate(-mesh_->GetCenter());
        mesh_->ComputeVertexNormals();
        mesh_->PaintUniformColor(Eigen::Vector3d(0.7, 0.7, 0.9));
    }
    
    original_vertices_ = mesh_->vertices_;
    coord_frame_ = geometry::TriangleMesh::CreateCoordinateFrame(0.3);
}
void IMUVisualizer::SetupUI() {
    auto& theme = vis_->GetTheme();
    int em = theme.font_size;
    int left_margin = em;
    int vspacing = em / 2;
    
    // Create main settings panel
    settings_panel_ = std::make_shared<gui::Vert>(0, gui::Margins(left_margin));
    
    // Title
    auto title = std::make_shared<gui::Label>("BNO085 IMU Controls");
    settings_panel_->AddChild(title);
    settings_panel_->AddFixed(vspacing);
    settings_panel_->SetFrame(gui::Rect(0, 24, 300, 600));
    
    // Sensor Type Dropdown
    auto sensor_label = std::make_shared<gui::Label>("Sensor Type:");
    settings_panel_->AddChild(sensor_label);
    
    sensor_type_dropdown_ = std::make_shared<gui::Combobox>();
    for (const auto& sensor_pair : sensor_types_) {
        sensor_type_dropdown_->AddItem(sensor_pair.first.c_str());
    }
    sensor_type_dropdown_->SetSelectedIndex(0);
    sensor_type_dropdown_->SetOnValueChanged([this](const char* text, int index) {
        OnSensorTypeChanged(text, index);
    });
    settings_panel_->AddChild(sensor_type_dropdown_);
    settings_panel_->AddFixed(vspacing);
    
    // Sampling Rate Dropdown
    auto rate_label = std::make_shared<gui::Label>("Sampling Rate:");
    settings_panel_->AddChild(rate_label);
    
    sampling_rate_dropdown_ = std::make_shared<gui::Combobox>();
    for (const auto& rate_pair : sampling_rates_) {
        sampling_rate_dropdown_->AddItem(rate_pair.first.c_str());
    }
    sampling_rate_dropdown_->SetSelectedIndex(5); // Default to 100 Hz
    sampling_rate_dropdown_->SetOnValueChanged([this](const char* text, int index) {
        if (index >= 0 && index < static_cast<int>(sampling_rates_.size())) {
            OnSamplingRateChanged(text, sampling_rates_[index].second);
        }
    });
    settings_panel_->AddChild(sampling_rate_dropdown_);
    settings_panel_->AddFixed(vspacing);
    
    // Enable Sensor Checkbox
    enable_sensor_checkbox_ = std::make_shared<gui::Checkbox>("Enable Sensor");
    enable_sensor_checkbox_->SetOnChecked([this](bool checked) {
        OnEnableSensorToggled(checked);
    });
    settings_panel_->AddChild(enable_sensor_checkbox_);
    settings_panel_->AddFixed(vspacing * 2);
    
    // Calibration Section
    auto cal_label = std::make_shared<gui::Label>("Calibration");
    settings_panel_->AddChild(cal_label);
    
    calibration_label_ = std::make_shared<gui::Label>("Status: Unknown");
    settings_panel_->AddChild(calibration_label_);
    
    calibrate_button_ = std::make_shared<gui::Button>("Start Calibration");
    calibrate_button_->SetOnClicked([this]() {
        OnCalibrateClicked();
    });
    settings_panel_->AddChild(calibrate_button_);
    
    save_cal_button_ = std::make_shared<gui::Button>("Save Calibration");
    save_cal_button_->SetOnClicked([this]() {
        OnSaveCalibrationClicked();
    });
    settings_panel_->AddChild(save_cal_button_);
    settings_panel_->AddFixed(vspacing * 2);
    
    // =========================================================================
    // TABBED SENSOR DATA DISPLAY
    // =========================================================================
    
    auto data_section_label = std::make_shared<gui::Label>("Sensor Data");
    settings_panel_->AddChild(data_section_label);
    settings_panel_->AddFixed(vspacing);
    
    // Create tab control
    data_tabs_ = std::make_shared<gui::TabControl>();
    
    // -------------------------------------------------------------------------
    // TAB 1: Current Sensor Data (Accel, Gyro, Mag)
    // -------------------------------------------------------------------------
    auto current_data_panel = std::make_shared<gui::Vert>(0, gui::Margins(em/2));
    
    auto accel_header = std::make_shared<gui::Label>("Accelerometer (m/s²)");
    current_data_panel->AddChild(accel_header);
    
    accel_label_ = std::make_shared<gui::Label>("X: 0.00  Y: 0.00  Z: 0.00");
    current_data_panel->AddChild(accel_label_);
    current_data_panel->AddFixed(vspacing);
    
    auto gyro_header = std::make_shared<gui::Label>("Gyroscope (rad/s)");
    current_data_panel->AddChild(gyro_header);
    
    gyro_label_ = std::make_shared<gui::Label>("X: 0.00  Y: 0.00  Z: 0.00");
    current_data_panel->AddChild(gyro_label_);
    current_data_panel->AddFixed(vspacing);
    
    auto mag_header = std::make_shared<gui::Label>("Magnetometer (µT)");
    current_data_panel->AddChild(mag_header);
    
    mag_label_ = std::make_shared<gui::Label>("X: 0.00  Y: 0.00  Z: 0.00");
    current_data_panel->AddChild(mag_label_);
    
    data_tabs_->AddTab("Raw Data", current_data_panel);
    
    // -------------------------------------------------------------------------
    // TAB 2: Euler Angles (Yaw, Pitch, Roll)
    // -------------------------------------------------------------------------
    auto euler_panel = std::make_shared<gui::Vert>(0, gui::Margins(em/2));
    
    auto yaw_header = std::make_shared<gui::Label>("Yaw (°)");
    euler_panel->AddChild(yaw_header);
    yaw_label_ = std::make_shared<gui::Label>("0.00");
    euler_panel->AddChild(yaw_label_);
    euler_panel->AddFixed(vspacing);
    
    auto pitch_header = std::make_shared<gui::Label>("Pitch (°)");
    euler_panel->AddChild(pitch_header);
    pitch_label_ = std::make_shared<gui::Label>("0.00");
    euler_panel->AddChild(pitch_label_);
    euler_panel->AddFixed(vspacing);
    
    auto roll_header = std::make_shared<gui::Label>("Roll (°)");
    euler_panel->AddChild(roll_header);
    roll_label_ = std::make_shared<gui::Label>("0.00");
    euler_panel->AddChild(roll_label_);
    
    data_tabs_->AddTab("Euler", euler_panel);
    
    // -------------------------------------------------------------------------
    // TAB 3: Quaternion
    // -------------------------------------------------------------------------
    auto quat_panel = std::make_shared<gui::Vert>(0, gui::Margins(em/2));
    
    auto quat_w_header = std::make_shared<gui::Label>("W (Real)");
    quat_panel->AddChild(quat_w_header);
    quat_w_label_ = std::make_shared<gui::Label>("1.000");
    quat_panel->AddChild(quat_w_label_);
    quat_panel->AddFixed(vspacing);
    
    auto quat_x_header = std::make_shared<gui::Label>("X (i)");
    quat_panel->AddChild(quat_x_header);
    quat_x_label_ = std::make_shared<gui::Label>("0.000");
    quat_panel->AddChild(quat_x_label_);
    quat_panel->AddFixed(vspacing);
    
    auto quat_y_header = std::make_shared<gui::Label>("Y (j)");
    quat_panel->AddChild(quat_y_header);
    quat_y_label_ = std::make_shared<gui::Label>("0.000");
    quat_panel->AddChild(quat_y_label_);
    quat_panel->AddFixed(vspacing);
    
    auto quat_z_header = std::make_shared<gui::Label>("Z (k)");
    quat_panel->AddChild(quat_z_header);
    quat_z_label_ = std::make_shared<gui::Label>("0.000");
    quat_panel->AddChild(quat_z_label_);
    quat_panel->AddFixed(vspacing);
    
    auto quat_acc_header = std::make_shared<gui::Label>("Accuracy (rad)");
    quat_panel->AddChild(quat_acc_header);
    quat_accuracy_label_ = std::make_shared<gui::Label>("0.000");
    quat_panel->AddChild(quat_accuracy_label_);
    
    data_tabs_->AddTab("Quaternion", quat_panel);
    
    // Add tab control to main panel
    settings_panel_->AddChild(data_tabs_);
    settings_panel_->AddFixed(vspacing * 2);
    
    // =========================================================================
    // END TABBED SECTION
    // =========================================================================
    
    // Reset Button
    reset_button_ = std::make_shared<gui::Button>("Reset Sensor");
    reset_button_->SetOnClicked([this]() {
        OnResetSensorClicked();
    });
    settings_panel_->AddChild(reset_button_);
    
    // Add panel to visualizer
    vis_->AddChild(settings_panel_);
}

void IMUVisualizer::InitializeScene() {
    // Setup material
    auto mat = rendering::MaterialRecord();
    mat.shader = "defaultLit";
    
    // Add geometries
    vis_->AddGeometry("mesh", mesh_, &mat);
    vis_->AddGeometry("axes", coord_frame_, &mat);
    
    // Setup camera
    auto bounds = mesh_->GetAxisAlignedBoundingBox();
    Eigen::Vector3f center = bounds.GetCenter().cast<float>();
    Eigen::Vector3f eye = center + Eigen::Vector3f(2.0f, 2.0f, 2.0f);
    vis_->SetupCamera(60.0f, center, eye, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    
    // Setup UI
    SetupUI();
    
    scene_initialized_ = true;
}

void IMUVisualizer::Run() {
    is_running_ = true;
    
    // Create visualizer
    vis_ = std::make_shared<visualizer::O3DVisualizer>(
        "BNO085 IMU Visualizer", 1600, 900);
    
    vis_->SetOnClose([this]() {
        is_running_ = false;
        return true;
    });
    
    // Add window
    gui::Application::GetInstance().AddWindow(vis_);
    
    // Schedule scene initialization on main thread
    gui::Application::GetInstance().PostToMainThread(
        vis_.get(), [this]() {
            InitializeScene();
        });
}

void IMUVisualizer::UpdateSensorData(const BNO085::Quaternion& quat,
                                     const BNO085::Vector3& accel) {
    if (!is_running_ || !vis_ || !scene_initialized_) {
        return;
    }
    
    gui::Application::GetInstance().PostToMainThread(
        vis_.get(), [this, quat, accel]() {
            if (!is_running_ || !scene_initialized_) return;
            
            // Create transform matrix
            Eigen::Quaterniond eigen_quat(quat.w, quat.x, quat.y, quat.z);
            Eigen::Matrix3d rotation = eigen_quat.normalized().toRotationMatrix();
            
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform.block<3,3>(0,0) = rotation;
            
            // Update geometry transform
            vis_->GetScene()->SetGeometryTransform("mesh", transform);
            
            // Update window title with orientation
            BNO085::Vector3 euler = quat.toEulerAngles();
            char buffer[256];
            snprintf(buffer, sizeof(buffer), 
                     "BNO085 IMU | Y:%.1f° P:%.1f° R:%.1f° | Acc:(%.2f, %.2f, %.2f)",
                     euler.x, euler.y, euler.z, accel.x, accel.y, accel.z);
            vis_->SetTitle(buffer);
            
            // Update sensor info label
            UpdateSensorInfo();
            UpdateCalibrationStatus();
        });
}

// ============================================================================
// UI Callback Implementations
// ============================================================================

void IMUVisualizer::OnSensorTypeChanged(const char* sensor_name, int index) {
    std::cout << "Sensor type changed to: " << sensor_name << " (index: " << index << ")" << std::endl;
    current_sensor_index_ = index;
    
    // Update UI to reflect current sensor's configuration
    BNO085::SensorType sensor_type = GetCurrentSensorType();
    auto& config = sensor_configs_[sensor_type];
    
    enable_sensor_checkbox_->SetChecked(config.enabled);
    
    // Find matching sampling rate index
    float current_freq = config.getFrequency();
    for (size_t i = 0; i < sampling_rates_.size(); ++i) {
        if (std::abs(sampling_rates_[i].second - current_freq) < 0.1f) {
            sampling_rate_dropdown_->SetSelectedIndex(i);
            break;
        }
    }
}

void IMUVisualizer::OnSamplingRateChanged(const char* label, double value) {
    std::cout << "Sampling rate changed to: " << label << " (" << value << " Hz)" << std::endl;
    
    BNO085::SensorType sensor_type = GetCurrentSensorType();
    sensor_configs_[sensor_type].setFrequency(static_cast<float>(value));
    
    // If sensor is currently enabled, re-apply configuration
    if (sensor_configs_[sensor_type].enabled) {
        ApplySensorConfiguration();
    }
}

void IMUVisualizer::OnEnableSensorToggled(bool enabled) {
    BNO085::SensorType sensor_type = GetCurrentSensorType();
    sensor_configs_[sensor_type].enabled = enabled;
    
    std::cout << "Sensor " << sensor_types_[current_sensor_index_].first 
              << (enabled ? " enabled" : " disabled") << std::endl;
    
    if (enabled) {
        ApplySensorConfiguration();
    } else {
        sensor_.disableSensor(sensor_type);
    }
}

void IMUVisualizer::OnCalibrateClicked() {
    std::cout << "Starting calibration..." << std::endl;
    
    BNO085::SensorType sensor_type = GetCurrentSensorType();
    auto& config = sensor_configs_[sensor_type];
    
    if (sensor_.startCalibration(config.reportInterval_us)) {
        calibrate_button_->SetText("Stop Calibration");
        std::cout << "Calibration started. Move the sensor in a figure-8 pattern." << std::endl;
    } else {
        std::cerr << "Failed to start calibration" << std::endl;
    }
}

void IMUVisualizer::OnSaveCalibrationClicked() {
    std::cout << "Saving calibration..." << std::endl;
    
    if (sensor_.saveCalibration()) {
        std::cout << "Calibration saved successfully!" << std::endl;
    } else {
        std::cerr << "Failed to save calibration" << std::endl;
    }
}

void IMUVisualizer::OnResetSensorClicked() {
    std::cout << "Resetting sensor..." << std::endl;
    
    sensor_.hardwareReset();
    if (sensor_.hardwareReset()) {
        std::cout << "Sensor reset successfully!" << std::endl;
    
    } else {
        std::cerr << "Failed to reset sensor" << std::endl;
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

BNO085::SensorType IMUVisualizer::GetCurrentSensorType() const {
    if (current_sensor_index_ >= 0 && 
        current_sensor_index_ < static_cast<int>(sensor_types_.size())) {
        return sensor_types_[current_sensor_index_].second;
    }
    return BNO085::SensorType::ROTATION_VECTOR;
}

void IMUVisualizer::ApplySensorConfiguration() {
    BNO085::SensorType sensor_type = GetCurrentSensorType();
    auto& config = sensor_configs_[sensor_type];
    
    if (sensor_.enableSensor(sensor_type, config)) {
        std::cout << "Applied configuration: " << sensor_types_[current_sensor_index_].first
                  << " at " << config.getFrequency() << " Hz" << std::endl;
    } else {
        std::cerr << "Failed to apply sensor configuration" << std::endl;
    }
}

void IMUVisualizer::UpdateCalibrationStatus() {
    if (!calibration_label_) return;
    
    BNO085::CalibrationStatus accel, gyro, mag, system;
    sensor_.getCalibrationStatus(accel, gyro, mag, system);
    
    std::ostringstream oss;
    oss << "Accel:" << static_cast<int>(accel) 
        << " Gyro:" << static_cast<int>(gyro)
        << " Mag:" << static_cast<int>(mag)
        << " Sys:" << static_cast<int>(system);
    
    calibration_label_->SetText(oss.str().c_str());
}

void IMUVisualizer::UpdateSensorInfo() {
    if (!accel_label_ || !gyro_label_ || !mag_label_) return;
    
    BNO085::IMUData imu_data;
    BNO085::OrientationData orient_data;
    
    // Update Raw Data Tab
    if (sensor_.getIMUData(imu_data)) {
        std::ostringstream oss;
        
        // Accelerometer
        oss.str("");
        oss << std::fixed << std::setprecision(2);
        oss << "X: " << std::setw(6) << imu_data.acceleration.x << "  "
            << "Y: " << std::setw(6) << imu_data.acceleration.y << "  "
            << "Z: " << std::setw(6) << imu_data.acceleration.z;
        accel_label_->SetText(oss.str().c_str());
        
        // Gyroscope
        oss.str("");
        oss << "X: " << std::setw(6) << imu_data.gyroscope.x << "  "
            << "Y: " << std::setw(6) << imu_data.gyroscope.y << "  "
            << "Z: " << std::setw(6) << imu_data.gyroscope.z;
        gyro_label_->SetText(oss.str().c_str());
        
        // Magnetometer
        oss.str("");
        oss << "X: " << std::setw(6) << imu_data.magnetometer.x << "  "
            << "Y: " << std::setw(6) << imu_data.magnetometer.y << "  "
            << "Z: " << std::setw(6) << imu_data.magnetometer.z;
        mag_label_->SetText(oss.str().c_str());
    }
    
    // Update Euler Angles Tab
    if (sensor_.getOrientationData(orient_data)) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        
        BNO085::Vector3 euler = orient_data.rotation.toEulerAngles();
        
        oss.str("");
        oss << std::setw(7) << euler.x << "°";
        yaw_label_->SetText(oss.str().c_str());
        
        oss.str("");
        oss << std::setw(7) << euler.y << "°";
        pitch_label_->SetText(oss.str().c_str());
        
        oss.str("");
        oss << std::setw(7) << euler.z << "°";
        roll_label_->SetText(oss.str().c_str());
        
        // Update Quaternion Tab
        oss.str("");
        oss << std::setprecision(3);
        oss << std::setw(7) << orient_data.rotation.w;
        quat_w_label_->SetText(oss.str().c_str());
        
        oss.str("");
        oss << std::setw(7) << orient_data.rotation.x;
        quat_x_label_->SetText(oss.str().c_str());
        
        oss.str("");
        oss << std::setw(7) << orient_data.rotation.y;
        quat_y_label_->SetText(oss.str().c_str());
        
        oss.str("");
        oss << std::setw(7) << orient_data.rotation.z;
        quat_z_label_->SetText(oss.str().c_str());
        
        oss.str("");
        oss << std::setw(7) << orient_data.rotation.accuracy;
        quat_accuracy_label_->SetText(oss.str().c_str());
    }
}
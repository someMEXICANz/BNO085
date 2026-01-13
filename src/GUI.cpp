// GUI.cpp - Fixed version
#include "GUI.h"
#include <iostream>

using namespace open3d;
using namespace open3d::visualization;

IMUVisualizer::IMUVisualizer(const std::string& model_path)
    : is_running_(false), scene_initialized_(false), data_available_(false) {
    
    current_quat_ = BNO085::Quaternion(1, 0, 0, 0);
    current_accel_ = BNO085::Vector3(0, 0, 0);
    
    LoadModel(model_path);
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

void IMUVisualizer::InitializeScene() {
    // Setup material
    auto mat = rendering::MaterialRecord();
    mat.shader = "defaultLit";
    
    // Add geometries (only once!)
    vis_->AddGeometry("mesh", mesh_, &mat);
    vis_->AddGeometry("axes", coord_frame_, &mat);
    
    // Setup camera
    auto bounds = mesh_->GetAxisAlignedBoundingBox();
    Eigen::Vector3f center = bounds.GetCenter().cast<float>();
    Eigen::Vector3f eye = center + Eigen::Vector3f(2.0f, 2.0f, 2.0f);
    vis_->SetupCamera(60.0f, center, eye, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    
    scene_initialized_ = true;
}

void IMUVisualizer::Run() {
    is_running_ = true;
    
    // Create visualizer
    vis_ = std::make_shared<visualizer::O3DVisualizer>(
        "BNO085 IMU Visualizer", 1280, 800);
    
    vis_->SetOnClose([this]() {
        is_running_ = false;
        return true;
    });
    
    // Add window and initialize scene on main thread
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
            
            // Update window title
            BNO085::Vector3 euler = quat.toEulerAngles();
            char buffer[256];
            snprintf(buffer, sizeof(buffer), 
                     "BNO085 IMU | Y:%.1f° P:%.1f° R:%.1f° | Acc:(%.2f, %.2f, %.2f)",
                     euler.x, euler.y, euler.z, accel.x, accel.y, accel.z);
            vis_->SetTitle(buffer);
        });
}
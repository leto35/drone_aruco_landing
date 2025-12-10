/**
 * @file my_drone_pid.cpp
 * @brief Autonomous landing on moving ArUco target with adaptive PID
 */

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#ifdef HAVE_GZ
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#endif

using namespace mavsdk;

#ifdef HAVE_GZ
class CameraHandler {
public:
    CameraHandler(const std::string& topic) {
        if (!node.Subscribe(topic, &CameraHandler::image_callback, this)) {
            throw std::runtime_error("Failed to connect camera");
        }
        std::cout << "Camera feed connected successfully" << std::endl;
    }

    bool retrieve_frame(cv::Mat& output) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (current_frame_.empty()) return false;
        output = current_frame_.clone();
        return true;
    }

private:
    void image_callback(const gz::msgs::Image& msg) {
        if (msg.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) return;

        cv::Mat temp_img(msg.height(), msg.width(), CV_8UC3);
        memcpy(temp_img.data, msg.data().c_str(), msg.data().size());
        cv::cvtColor(temp_img, temp_img, cv::COLOR_RGB2BGR);

        std::lock_guard<std::mutex> lock(frame_mutex_);
        current_frame_ = temp_img;
    }

    gz::transport::Node node;
    cv::Mat current_frame_;
    std::mutex frame_mutex_;
};
#endif

class ArucoDetector {
public:
    ArucoDetector() {
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detect_params_ = cv::aruco::DetectorParameters::create();

        detect_params_->minMarkerPerimeterRate = 0.01;
        detect_params_->maxMarkerPerimeterRate = 4.0;
        detect_params_->adaptiveThreshWinSizeMin = 3;
        detect_params_->adaptiveThreshWinSizeMax = 23;
        detect_params_->adaptiveThreshConstant = 7;
        detect_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detect_params_->cornerRefinementWinSize = 5;

        std::cout << "ArUco marker detector initialized" << std::endl;
    }

    bool detect(const cv::Mat& image, cv::Point2f& position, int& id) {
        std::vector<int> detected_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

        cv::aruco::detectMarkers(image, aruco_dict_, marker_corners, detected_ids, detect_params_, rejected_candidates);

        if (detected_ids.empty()) return false;

        id = detected_ids[0];

        position = cv::Point2f(0, 0);
        for (const auto& pt : marker_corners[0]) {
            position += pt;
        }
        position /= 4.0f;

        stored_corners_ = marker_corners;
        stored_ids_ = detected_ids;

        return true;
    }

    void draw_markers(cv::Mat& image, const cv::Point2f& target_pos, int id) {
        if (!stored_ids_.empty()) {
            cv::aruco::drawDetectedMarkers(image, stored_corners_, stored_ids_);
        }

        cv::Point2f center_point(image.cols/2.0f, image.rows/2.0f);
        cv::circle(image, center_point, 5, cv::Scalar(0,255,0), -1);
        cv::circle(image, target_pos, 5, cv::Scalar(0,0,255), -1);
        cv::line(image, center_point, target_pos, cv::Scalar(255,0,0), 2);
    }

private:
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detect_params_;
    std::vector<std::vector<cv::Point2f>> stored_corners_;
    std::vector<int> stored_ids_;
};

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_integral = 2.0f) 
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), 
          error_sum_(0), last_error_(0) {}

    float compute(float error, float dt) {
        // Terme P (Proportionnel) - Réactivité immédiate
        float p_term = kp_ * error;

        // Terme I (Intégral) - Correction des erreurs persistantes
        error_sum_ += error * dt;
        error_sum_ = std::clamp(error_sum_, -max_integral_, max_integral_);
        float i_term = ki_ * error_sum_;

        // Terme D (Dérivé) - Anticipation et stabilité
        float d_term = 0.0f;
        if (dt > 0) {
            d_term = kd_ * (error - last_error_) / dt;
        }
        last_error_ = error;

        return p_term + i_term + d_term;
    }

    void reset() {
        error_sum_ = 0;
        last_error_ = 0;
    }

    void set_gains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    float kp_, ki_, kd_;
    float max_integral_;
    float error_sum_;
    float last_error_;
};

class DroneController {
public:
    DroneController(std::shared_ptr<System> sys) : system_(sys) {
        action = std::make_unique<Action>(sys);
        telemetry = std::make_unique<Telemetry>(sys);
        offboard = std::make_unique<Offboard>(sys);
        gimbal = std::make_unique<Gimbal>(sys);

        // PID avec gains DOUX : P=2.0, I=0.5, D=0.3
        pid_x_ = std::make_unique<PIDController>(2.0f, 0.5f, 0.3f);
        pid_y_ = std::make_unique<PIDController>(2.0f, 0.5f, 0.3f);

        last_time_ = std::chrono::steady_clock::now();
    }

    bool execute_takeoff() {
        std::cout << "=== INITIATING TAKEOFF SEQUENCE ===" << std::endl;

        while (!telemetry->health_all_ok()) {
            std::cout << "Waiting for drone readiness..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (action->arm() != Action::Result::Success) {
            std::cerr << "Failed to arm motors" << std::endl;
            return false;
        }

        if (action->takeoff() != Action::Result::Success) {
            std::cerr << "Takeoff command failed" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(8));

        std::cout << "Climbing to operational altitude..." << std::endl;
        Action::Result climb_result = action->goto_location(
            telemetry->position().latitude_deg,
            telemetry->position().longitude_deg,
            10.0f,
            0.0f
        );

        if (climb_result == Action::Result::Success) {
            std::cout << "Ascending to 10m target altitude..." << std::endl;
            while (telemetry->position().relative_altitude_m < 9.0f) {
                std::cout << "Current height: " << telemetry->position().relative_altitude_m << "m" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "Target altitude reached!" << std::endl;
        } else {
            std::cerr << "Manual climb error" << std::endl;
        }

        while (telemetry->flight_mode() != Telemetry::FlightMode::Hold) {
            std::cout << "Waiting for position stabilization..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "Drone stabilized at initial position" << std::endl;

        gimbal->take_control(0, Gimbal::ControlMode::Primary);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gimbal->set_angles(0, 0.0f, -90.0f, 0.0f, Gimbal::GimbalMode::YawLock, Gimbal::SendMode::Once);

        std::cout << "Takeoff complete - camera pointing downward" << std::endl;
        return true;
    }

    void control_flight(const cv::Point2f& aruco_center, const cv::Size& frame_size, bool aruco_detected) {
        }

        if (action->takeoff() != Action::Result::Success) {
            std::cerr << "Takeoff command failed" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(8));

        std::cout << "Climbing to operational altitude..." << std::endl;
        Action::Result climb_result = action->goto_location(
            telemetry->position().latitude_deg,
            telemetry->position().longitude_deg,
            10.0f,
            0.0f
        );

        if (climb_result == Action::Result::Success) {
            std::cout << "Ascending to 10m target altitude..." << std::endl;
            while (telemetry->position().relative_altitude_m < 9.0f) {
                std::cout << "Current height: " << telemetry->position().relative_altitude_m << "m" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "Target altitude reached!" << std::endl;
        } else {
            std::cerr << "Manual climb error" << std::endl;
        }

        while (telemetry->flight_mode() != Telemetry::FlightMode::Hold) {
            std::cout << "Waiting for position stabilization..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "Drone stabilized at initial position" << std::endl;

        gimbal->take_control(0, Gimbal::ControlMode::Primary);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gimbal->set_angles(0, 0.0f, -90.0f, 0.0f, Gimbal::GimbalMode::YawLock, Gimbal::SendMode::Once);

        std::cout << "Takeoff complete - camera pointing downward" << std::endl;
        return true;
    }

    bool start_offboard() {
        Offboard::VelocityNedYaw stay{};
        offboard->set_velocity_ned(stay);

        Offboard::Result offboard_result = offboard->start();
        if (offboard_result != Offboard::Result::Success) {
            std::cerr << "Erreur mode offboard" << std::endl;
            return false;
        }

        std::cout << "Mode offboard activé" << std::endl;
        return true;
    }

private:
    std::unique_ptr<Action> action;
    std::unique_ptr<Telemetry> telemetry;
    std::unique_ptr<Offboard> offboard;
    std::unique_ptr<Gimbal> gimbal;
};

int main() {
    std::cout << "=== AUTONOMOUS ARUCO LANDING SYSTEM ===" << std::endl;

    Mavsdk mavsdk(Mavsdk::Configuration(ComponentType::GroundStation));
    mavsdk.add_any_connection("udpin://0.0.0.0:14540");

    std::shared_ptr<System> drone_system;
    while (!drone_system) {
        auto available_systems = mavsdk.systems();
        if (!available_systems.empty()) drone_system = available_systems[0];
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Drone connection established!" << std::endl;

#ifdef HAVE_GZ
    std::unique_ptr<CameraHandler> camera_feed;
    try {
        camera_feed = std::make_unique<CameraHandler>("/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image");
    } catch (const std::exception& error) {
        std::cerr << "Camera error: " << error.what() << std::endl;
        return -1;
    ArucoDetector detector;
    DroneController drone(drone_system);

    if (!drone.execute_takeoff()) {
        return -1;
    }

    if (!drone.start_offboard()) {
        return -1;
    }

    std::cout << "=== BEGINNING ARUCO TRACKING AND LANDING ===" << std::endl;

    while (true) {
        cv::Mat frame;
        if (!camera_feed->retrieve_frame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Point2f marker_center;
        int marker_id;
        bool marker_detected = detector.detect(frame, marker_center, marker_id);

        // Contrôle de vol avec PID complet
        drone.control_flight(marker_center, frame.size(), marker_detected);

        // Affichage visuel
        if (marker_detected) {
            detector.draw_markers(frame, marker_center, marker_id);
        }

        cv::imshow("Atterrissage PID Mobile", frame);
        
        if (cv::waitKey(1) == 27 || drone.is_landing_complete()) {
            break;
        }

        // Boucle à 10 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }   auto iteration_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(iteration_end - iteration_start);
        auto sleep_time = std::chrono::milliseconds(100) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    cv::destroyAllWindows();
    std::cout << "Mission completed successfully" << std::endl;
    return 0;
}
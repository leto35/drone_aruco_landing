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

class MarkerDetector {
public:
    MarkerDetector() {
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

    bool find_marker(const cv::Mat& image, cv::Point2f& position, int& id) {
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

    void visualize_detection(cv::Mat& image, const cv::Point2f& target_pos, int id) {
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

class AdaptiveController {
public:
    AdaptiveController(float p_gain, float i_gain, float d_gain, float integral_limit = 2.0f)
        : kp_(p_gain), ki_(i_gain), kd_(d_gain), int_limit_(integral_limit),
          integral_error_(0), prev_error_(0) {}

    float calculate(float current_error, float delta_time) {
        float proportional = kp_ * current_error;

        integral_error_ += current_error * delta_time;
        integral_error_ = std::clamp(integral_error_, -int_limit_, int_limit_);
        float integral = ki_ * integral_error_;

        float derivative = 0.0f;
        if (delta_time > 0) {
            derivative = kd_ * (current_error - prev_error_) / delta_time;
        }
        prev_error_ = current_error;

        return proportional + integral + derivative;
    }

    void clear() {
        integral_error_ = 0;
        prev_error_ = 0;
    }

    void adjust_gains(float p, float i, float d) {
        kp_ = p;
        ki_ = i;
        kd_ = d;
    }

private:
    float kp_, ki_, kd_;
    float int_limit_;
    float integral_error_;
    float prev_error_;
};

class FlightManager {
public:
    FlightManager(std::shared_ptr<System> sys) : drone_system_(sys) {
        action_ = std::make_unique<Action>(sys);
        telemetry_ = std::make_unique<Telemetry>(sys);
        offboard_ = std::make_unique<Offboard>(sys);
        gimbal_ = std::make_unique<Gimbal>(sys);

        lateral_pid_ = std::make_unique<AdaptiveController>(2.0f, 0.5f, 0.3f);
        longitudinal_pid_ = std::make_unique<AdaptiveController>(2.0f, 0.5f, 0.3f);

        previous_timestamp_ = std::chrono::steady_clock::now();
    }

    bool execute_takeoff() {
        std::cout << "=== INITIATING TAKEOFF SEQUENCE ===" << std::endl;

        while (!telemetry_->health_all_ok()) {
            std::cout << "Waiting for drone readiness..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (action_->arm() != Action::Result::Success) {
            std::cerr << "Failed to arm motors" << std::endl;
            return false;
        }

        if (action_->takeoff() != Action::Result::Success) {
            std::cerr << "Takeoff command failed" << std::endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(8));

        std::cout << "Climbing to operational altitude..." << std::endl;
        Action::Result climb_result = action_->goto_location(
            telemetry_->position().latitude_deg,
            telemetry_->position().longitude_deg,
            10.0f,
            0.0f
        );

        if (climb_result == Action::Result::Success) {
            std::cout << "Ascending to 10m target altitude..." << std::endl;
            while (telemetry_->position().relative_altitude_m < 9.0f) {
                std::cout << "Current height: " << telemetry_->position().relative_altitude_m << "m" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "Target altitude reached!" << std::endl;
        } else {
            std::cerr << "Manual climb error" << std::endl;
        }

        while (telemetry_->flight_mode() != Telemetry::FlightMode::Hold) {
            std::cout << "Waiting for position stabilization..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "Drone stabilized at initial position" << std::endl;

        gimbal_->take_control(0, Gimbal::ControlMode::Primary);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        gimbal_->set_angles(0, 0.0f, -90.0f, 0.0f, Gimbal::GimbalMode::YawLock, Gimbal::SendMode::Once);

        std::cout << "Takeoff complete - camera pointing downward" << std::endl;
        return true;
    }

    bool activate_offboard_mode() {
        std::cout << "=== ENABLING OFFBOARD CONTROL ===" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(2));

        Offboard::VelocityNedYaw initial_cmd{0, 0, 0, 0};
        for (int i = 0; i < 20; i++) {
            offboard_->set_velocity_ned(initial_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        for (int retry = 0; retry < 3; retry++) {
            auto activation_result = offboard_->start();
            if (activation_result == Offboard::Result::Success) {
                std::cout << "Offboard mode successfully activated" << std::endl;
                return true;
            }

            std::cout << "Attempt " << (retry + 1) << " unsuccessful, retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            for (int i = 0; i < 10; i++) {
                offboard_->set_velocity_ned(initial_cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::cerr << "Offboard activation failed after 3 attempts" << std::endl;
        return false;
    }

    void update_position(const cv::Point2f& target_center, const cv::Size& img_dimensions, bool target_found) {
        float vel_forward = 0, vel_right = 0, vel_descend = 0;
        float current_height = telemetry_->position().relative_altitude_m;

        auto current_timestamp = std::chrono::steady_clock::now();
        float time_delta = std::chrono::duration<float>(current_timestamp - previous_timestamp_).count();
        previous_timestamp_ = current_timestamp;

        if (target_found && current_height > 0.1f) {
            cv::Point2f image_center(img_dimensions.width/2.0f, img_dimensions.height/2.0f);
            cv::Point2f tracking_error = target_center - image_center;

            float normalized_x_error = tracking_error.x / (img_dimensions.width/2.0f);
            float normalized_y_error = tracking_error.y / (img_dimensions.height/2.0f);

            if (current_height > 2.0f) {
                lateral_pid_->adjust_gains(2.0f, 0.5f, 0.3f);
                longitudinal_pid_->adjust_gains(2.0f, 0.5f, 0.3f);
            } else {
                lateral_pid_->adjust_gains(1.5f, 0.3f, 0.2f);
                longitudinal_pid_->adjust_gains(1.5f, 0.3f, 0.2f);
            }

            float velocity_limit = (current_height > 2.0f) ? 2.0f : 1.5f;
            vel_right = std::clamp(lateral_pid_->calculate(normalized_x_error, time_delta), -velocity_limit, velocity_limit);
            vel_forward = std::clamp(-longitudinal_pid_->calculate(normalized_y_error, time_delta), -velocity_limit, velocity_limit);

            vel_descend = (current_height > 3.0f) ? 0.4f :
                         (current_height > 1.5f) ? 0.3f : 0.2f;

            std::cout << "PID TRACKING - Height: " << current_height << "m, Error: "
                      << normalized_x_error << "," << normalized_y_error
                      << ", Velocities: " << vel_right << "," << vel_forward << "," << vel_descend
                      << ", dt: " << time_delta << "s" << std::endl;

        } else if (current_height > 0.8f) {
            if (search_iterations_ == 0) {
                std::cout << "Target lost - Slow descent while searching" << std::endl;
                lateral_pid_->clear();
                longitudinal_pid_->clear();
            }

            vel_descend = 0.15f;

            if (++search_iterations_ % 50 == 0) {
                std::cout << "Searching for ArUco marker at " << current_height << "m" << std::endl;
            }

        } else {
            std::cout << "Final descent phase - Height: " << current_height << "m" << std::endl;
            lateral_pid_->clear();
            longitudinal_pid_->clear();
            vel_forward = 0;
            vel_right = 0;
            vel_descend = 0.5f;
        }

        if (target_found) {
            search_iterations_ = 0;
        }

        Offboard::VelocityNedYaw velocity_command{vel_forward, vel_right, vel_descend, 0};
        offboard_->set_velocity_ned(velocity_command);

        if (current_height < 0.15f) {
            std::cout << "AUTOMATIC LANDING INITIATED" << std::endl;
            action_->land();
            landing_complete_ = true;
        }
    }

    float current_altitude() {
        return telemetry_->position().relative_altitude_m;
    }

    bool is_landing_complete() {
        return landing_complete_;
    }

private:
    std::shared_ptr<System> drone_system_;
    std::unique_ptr<Action> action_;
    std::unique_ptr<Telemetry> telemetry_;
    std::unique_ptr<Offboard> offboard_;
    std::unique_ptr<Gimbal> gimbal_;

    std::unique_ptr<AdaptiveController> lateral_pid_;
    std::unique_ptr<AdaptiveController> longitudinal_pid_;
    std::chrono::steady_clock::time_point previous_timestamp_;

    int search_iterations_ = 0;
    bool landing_complete_ = false;

    float saved_vel_forward_ = 0.0f;
    float saved_vel_right_ = 0.0f;
    bool velocity_saved_ = false;
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
    }
#else
    std::cerr << "Gazebo support required!" << std::endl;
    return -1;
#endif

    MarkerDetector marker_tracker;
    FlightManager flight_control(drone_system);

    if (!flight_control.execute_takeoff()) {
        return -1;
    }

    if (!flight_control.activate_offboard_mode()) {
        return -1;
    }

    std::cout << "=== BEGINNING ARUCO TRACKING AND LANDING ===" << std::endl;

    while (true) {
        cv::Mat video_frame;
        if (!camera_feed->retrieve_frame(video_frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        cv::Point2f marker_position;
        int detected_id;
        bool marker_visible = marker_tracker.find_marker(video_frame, marker_position, detected_id);

        flight_control.update_position(marker_position, video_frame.size(), marker_visible);

        if (marker_visible) {
            marker_tracker.visualize_detection(video_frame, marker_position, detected_id);
            std::string display_info = "ArUco ID:" + std::to_string(detected_id) +
                              " Alt:" + std::to_string(flight_control.current_altitude()).substr(0,4) + "m (Adaptive PID)";
        } else {
        }

        cv::imshow("Autonomous Landing - Camera Feed", video_frame);

        if (cv::waitKey(1) == 27 || flight_control.is_landing_complete()) {
            break;
        }
    }

    cv::destroyAllWindows();
    std::cout << "Mission completed successfully" << std::endl;
    return 0;
}

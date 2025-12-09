/**
 * @file my_drone_fixed_target.cpp
 * @brief Autonomous landing on fixed ArUco target with simple proportional control
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

class SimpleProportionalController {
public:
    SimpleProportionalController(float kp) : kp_(kp) {}
    
    float calculate(float error) {
        return kp_ * error;
    }
    
private:
    float kp_;
};

class FlightManager {
public:
    FlightManager(std::shared_ptr<System> sys) : drone_system_(sys) {
        action_ = std::make_unique<Action>(sys);
        telemetry_ = std::make_unique<Telemetry>(sys);
        offboard_ = std::make_unique<Offboard>(sys);
        gimbal_ = std::make_unique<Gimbal>(sys);
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
            8.0f,
            0.0f
        );

        if (climb_result == Action::Result::Success) {
            std::cout << "Ascending to 8m target altitude..." << std::endl;
            while (telemetry_->position().relative_altitude_m < 7.0f) {
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

    bool search_for_target(MarkerDetector& detector, CameraHandler& camera) {
        std::cout << "=== STARTING ZIGZAG SEARCH PATTERN ===" << std::endl;
        
        const int num_lines = 5;
        const float line_length = 20.0f;
        const float lateral_spacing = 4.0f;
        
        float initial_lat = telemetry_->position().latitude_deg;
        float initial_lon = telemetry_->position().longitude_deg;
        float altitude = telemetry_->position().relative_altitude_m;
        
        for (int line = 0; line < num_lines; line++) {
            std::cout << "Search line " << (line + 1) << "/" << num_lines << std::endl;
            
            // Calculate offset for current line
            float lateral_offset = line * lateral_spacing;
            
            // Move forward along the line
            float forward_direction = (line % 2 == 0) ? 1.0f : -1.0f;
            
            auto start_time = std::chrono::steady_clock::now();
            auto search_duration = std::chrono::seconds(10); // Time to traverse line_length at reasonable speed
            
            while (std::chrono::steady_clock::now() - start_time < search_duration) {
                // Check for marker every 200ms
                cv::Mat frame;
                if (camera.retrieve_frame(frame)) {
                    cv::Point2f marker_pos;
                    int marker_id;
                    if (detector.find_marker(frame, marker_pos, marker_id)) {
                        std::cout << "Marker detected during search! Stopping search pattern." << std::endl;
                        // Stop movement
                        Offboard::VelocityNedYaw stop_cmd{0, 0, 0, 0};
                        offboard_->set_velocity_ned(stop_cmd);
                        return true;
                    }
                }
                
                // Continue forward movement
                Offboard::VelocityNedYaw search_cmd{forward_direction * 2.0f, 0, 0, 0};
                offboard_->set_velocity_ned(search_cmd);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            
            // Stop at end of line
            Offboard::VelocityNedYaw stop_cmd{0, 0, 0, 0};
            offboard_->set_velocity_ned(stop_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Move laterally for next line (if not last line)
            if (line < num_lines - 1) {
                auto lateral_start = std::chrono::steady_clock::now();
                auto lateral_duration = std::chrono::seconds(2); // Time to move lateral_spacing meters
                
                while (std::chrono::steady_clock::now() - lateral_start < lateral_duration) {
                    Offboard::VelocityNedYaw lateral_cmd{0, 2.0f, 0, 0};
                    offboard_->set_velocity_ned(lateral_cmd);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                offboard_->set_velocity_ned(stop_cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        
        std::cout << "Search pattern complete - marker not found" << std::endl;
        return false;
    }

    void update_position(const cv::Point2f& target_center, const cv::Size& img_dimensions, bool target_found) {
        float vel_forward = 0, vel_right = 0, vel_descend = 0;
        float current_height = telemetry_->position().relative_altitude_m;

        if (target_found && current_height > 0.4f) {
            cv::Point2f image_center(img_dimensions.width/2.0f, img_dimensions.height/2.0f);
            
            float error_x = target_center.x - image_center.x;
            float error_y = target_center.y - image_center.y;
            
            float kp = 0.0008f;
            vel_right = std::clamp(kp * error_x, -0.4f, 0.4f);
            vel_forward = std::clamp(-kp * error_y, -0.4f, 0.4f);
            
            float total_error = std::sqrt(error_x * error_x + error_y * error_y);
            
            if (total_error < 15.0f) {
                vel_descend = 0.15f;
            } else {
                vel_descend = 0.0f;
            }
            
            std::cout << "PROPORTIONAL CONTROL - Height: " << current_height << "m, Error: ("
                      << error_x << ", " << error_y << ") = " << total_error << " px, Velocities: ("
                      << vel_right << ", " << vel_forward << ", " << vel_descend << ")" << std::endl;
        } else {
            vel_forward = 0;
            vel_right = 0;
            vel_descend = 0;
            
            if (!target_found) {
                std::cout << "Target not detected - holding position at " << current_height << "m" << std::endl;
            }
        }

        Offboard::VelocityNedYaw velocity_command{vel_forward, vel_right, vel_descend, 0};
        offboard_->set_velocity_ned(velocity_command);

        if (current_height < 0.4f && target_found) {
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

    bool landing_complete_ = false;
};

int main() {
    std::cout << "=== AUTONOMOUS FIXED TARGET ARUCO LANDING SYSTEM ===" << std::endl;

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

    std::cout << "=== CHECKING FOR ARUCO TARGET ===" << std::endl;
    
    // Check if marker is immediately visible
    bool marker_found = false;
    for (int i = 0; i < 10; i++) {
        cv::Mat test_frame;
        if (camera_feed->retrieve_frame(test_frame)) {
            cv::Point2f marker_pos;
            int marker_id;
            if (marker_tracker.find_marker(test_frame, marker_pos, marker_id)) {
                std::cout << "Marker detected immediately - starting landing procedure" << std::endl;
                marker_found = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // If marker not found, perform zigzag search
    if (!marker_found) {
        std::cout << "Marker not visible - initiating search pattern" << std::endl;
        marker_found = flight_control.search_for_target(marker_tracker, *camera_feed);
        
        if (!marker_found) {
            std::cout << "Warning: Marker not found after search. Proceeding with landing attempt..." << std::endl;
        }
    }

    std::cout << "=== BEGINNING ARUCO TRACKING AND LANDING ===" << std::endl;

    while (true) {
        auto loop_start = std::chrono::steady_clock::now();
        
        cv::Mat video_frame;
        if (!camera_feed->retrieve_frame(video_frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Point2f marker_position;
        int detected_id;
        bool marker_visible = marker_tracker.find_marker(video_frame, marker_position, detected_id);

        flight_control.update_position(marker_position, video_frame.size(), marker_visible);

        if (marker_visible) {
            marker_tracker.visualize_detection(video_frame, marker_position, detected_id);
            std::string display_info = "ArUco ID:" + std::to_string(detected_id) +
                              " Alt:" + std::to_string(flight_control.current_altitude()).substr(0,4) + "m (Simple P)";
            cv::putText(video_frame, display_info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("Fixed Target Landing - Camera Feed", video_frame);

        if (cv::waitKey(1) == 27 || flight_control.is_landing_complete()) {
            break;
        }
        
        if (flight_control.current_altitude() < 0.15f) {
            std::cout << "Ground level reached - exiting" << std::endl;
            break;
        }
        
        // Maintain 20 Hz loop rate (50ms per iteration)
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        auto remaining = std::chrono::milliseconds(50) - elapsed;
        if (remaining.count() > 0) {
            std::this_thread::sleep_for(remaining);
        }
    }

    cv::destroyAllWindows();
    std::cout << "Mission completed successfully" << std::endl;
    return 0;
}

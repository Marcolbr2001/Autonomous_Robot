#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

enum class RobotState { EXPLORING, TARGET_DETECTED, STOPPED };
RobotState state = RobotState::EXPLORING;

const char* stateToString(RobotState stato) {
  switch (stato) {
      case RobotState::EXPLORING: return "EXPLORING";
      case RobotState::TARGET_DETECTED: return "TARGET_DETECTED";
      case RobotState::STOPPED: return "STOPPED";
      default: return "UNKNOWN";
  }
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto cmd = cluon::getCommandlineArguments(argc, argv);
    std::cout << "main start";

    if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
        (0 == cmd.count("width")) || (0 == cmd.count("height"))) {
        std::cout << argv[0] << " attaches to a shared memory area containing an image." << std::endl;
        std::cout << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<shared memory> --width=<width> --height=<height>" << std::endl;
    } else {
        std::string const name{cmd["name"]};
        uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
        uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};
        bool const verbose{cmd.count("verbose") != 0};

        float const base_thrust{(cmd["base-thrust"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust"])) : 0.30f};
        float const kp{(cmd["kp"].size() != 0) ? static_cast<float>(std::stof(cmd["kp"])) : 0.003f};

        float const frontThreshold{(cmd["front-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["front-threshold"])) : 0.20f};
        float const leftThreshold{(cmd["left-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["left-threshold"])) : 0.15f};
        float const rightThreshold{(cmd["right-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["right-threshold"])) : 0.15f};

        float const area_min{(cmd["area_min"].size() != 0) ? static_cast<float>(std::stof(cmd["area_min"])) : 200};
        float const area_max{(cmd["area_max"].size() != 0) ? static_cast<float>(std::stof(cmd["area_max"])) : 7000};
        float const stopThreshold{(cmd["stop-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["stop-threshold"])) : 40.0f};

        float const ignorePercentage{(cmd["ignore-top"].size() != 0) ? std::stof(cmd["ignore-top"]) : 0.5f};

        uint32_t const H_min{(cmd["H_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["H_min"])) : 90};
        uint32_t const S_min{(cmd["S_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["S_min"])) : 30};
        uint32_t const V_min{(cmd["V_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["V_min"])) : 150};

        uint32_t const H_max{(cmd["H_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["H_max"])) : 120};
        uint32_t const S_max{(cmd["S_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["S_max"])) : 255};
        uint32_t const V_max{(cmd["V_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["V_max"])) : 255};

        float const base_thrust_left{(cmd["base-thrust-left"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust-left"])) : 0.30f};
        float const base_thrust_right{(cmd["base-thrust-right"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust-right"])) : 0.30f};

        float const base_thrust_right_turn{(cmd["base-thrust-right-turn"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust-right-turn"])) : 0.15f};
        float const base_thrust_left_turn{(cmd["base-thrust-left-turn"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust-left-turn"])) : 0.15f};

        uint32_t const stabilityThreshold{(cmd["stability-threshold"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["stability-threshold"])) : 5};

        uint32_t const random_turn_time{(cmd["random-turn-time"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["random-turn-time"])) : 20};
        float const random_turn_duration{(cmd["random-turn-duration"].size() != 0) ? static_cast<float>(std::stof(cmd["random-turn-duration"])) : 1.0f};

        srand(static_cast<unsigned int>(time(NULL)));

        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{name}};
        if (sharedMemory && sharedMemory->valid()) {
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

            std::mutex distancesMutex;
            float front{0}, rear{0}, left{0}, right{0};
            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env) {
                auto senderStamp = env.senderStamp();
                auto dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
                std::lock_guard<std::mutex> lck(distancesMutex);
                switch (senderStamp) {
                    case 0: front = dr.distance(); break;
                    case 2: rear = dr.distance(); break;
                    case 1: left = dr.distance(); break;
                    case 3: right = dr.distance(); break;
                }
            };
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

            cv::Point2f targetPosition(-1, -1);

            float leftThrust = 0.15f;
            float rightThrust = 0.15f;

            while (od4.isRunning()) {
                cv::Mat img;

                sharedMemory->wait();
                sharedMemory->lock();
                void* raw = sharedMemory->data();
                if (raw != nullptr) {
                    cv::Mat wrapped(height, width, CV_8UC3, reinterpret_cast<unsigned char*>(raw));
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                if (img.empty()) continue;

                // Target detection with top percentage ignored
                cv::Mat imgHSV, blueMask;
                cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV); // Converted from BGR to HSV
                cv::inRange(imgHSV, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), blueMask);
                
                int maskStart = static_cast<int>(ignorePercentage * blueMask.rows);
                cv::Mat roiMask = cv::Mat::zeros(blueMask.size(), CV_8UC1);
                roiMask(cv::Rect(0, maskStart, blueMask.cols, blueMask.rows - maskStart)) = 1;
                blueMask.setTo(0, 1 - roiMask);

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                for (auto &cnt : contours) {
                    double area = cv::contourArea(cnt);
                    if (area > area_min &&  area < area_max) {
                        cv::Moments m = cv::moments(cnt);
                        if (m.m00 > 0) {
                            targetPosition = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
                            //targetDetected = true;
                            if (state == RobotState::EXPLORING) {
                                state = RobotState::TARGET_DETECTED;
                            }
                            break;
                        }
                    }
                }
                switch (state) {
                    

                    case RobotState::EXPLORING: {
                      
                        static auto lastTurnTime = std::chrono::steady_clock::now();
                        static auto turnStartTime = std::chrono::steady_clock::now();
                        static auto avoidStartTime = std::chrono::steady_clock::now();
                    
                        static int direction = 0; // 0 = straight, -1 = right, 1 = left
                        static bool isTurningRandomly = false;
                        static bool isAvoidingObstacle = false;
                    
                        // Checking valid values
                        static uint32_t frontStableCount = 0, leftStableCount = 0, rightStableCount = 0;
                        static float frontPrev = 0, leftPrev = 0, rightPrev = 0;
                    
                        std::lock_guard<std::mutex> lck(distancesMutex);
                    
                        // Front obstacles
                        bool wallFront = false;
                        if (front < frontThreshold && front > 0.01f) {
                            if (std::abs(front - frontPrev) < 0.001f) {
                                frontStableCount++;
                            } else {
                                frontStableCount = 0;
                            }
                            frontPrev = front;
                            wallFront = (frontStableCount <= stabilityThreshold);
                        } else {
                            frontStableCount = 0;
                            wallFront = false;
                        }
                    
                        // Left obstacles
                        bool wallLeft = false;
                        if (left < leftThreshold && left > 0.01f) {
                            if (std::abs(left - leftPrev) < 0.001f) {
                                leftStableCount++;
                            } else {
                                leftStableCount = 0;
                            }
                            leftPrev = left;
                            wallLeft = (leftStableCount <= stabilityThreshold);
                        } else {
                            leftStableCount = 0;
                            wallLeft = false;
                        }
                    
                        // Right obstacles
                        bool wallRight = false;
                        if (right < rightThreshold && right > 0.01f) {
                            if (std::abs(right - rightPrev) < 0.001f) {
                                rightStableCount++;
                            } else {
                                rightStableCount = 0;
                            }
                            rightPrev = right;
                            wallRight = (rightStableCount <= stabilityThreshold);
                        } else {
                            rightStableCount = 0;
                            wallRight = false;
                        }
                    
                        auto now = std::chrono::steady_clock::now();
                    
                        // Avoiding obstacles
                        if (!isAvoidingObstacle && (wallFront || wallLeft || wallRight)) {
                            if (wallFront) {
                                direction = -1; // gira a sinistra
                                std::cout << "[Avoiding Front Wall]" << std::endl;
                            } else if (wallLeft) {
                                direction = 1; // gira a destra
                                std::cout << "[Avoiding Left Wall]" << std::endl;
                            } else if (wallRight) {
                                direction = -1; // gira a sinistra
                                std::cout << "[Avoiding Right Wall]" << std::endl;
                            }
                    
                            avoidStartTime = now;
                            isAvoidingObstacle = true;
                            isTurningRandomly = false;
                        }
                    
                        // Motor direction commands
                        if (direction == -1) {
                            leftThrust = -base_thrust_left_turn;
                            rightThrust = base_thrust_right_turn;
                        } else if (direction == 1) {
                            leftThrust = base_thrust_left_turn;
                            rightThrust = -base_thrust_right_turn;
                        } else {
                            leftThrust = base_thrust_left;
                            rightThrust = base_thrust_right;
                            std::cout << "[Going Straight]" << std::endl;
                        }
                    
                        // Avoiding finished
                        if (isAvoidingObstacle) {
                            float elapsedAvoid = std::chrono::duration_cast<std::chrono::milliseconds>(now - avoidStartTime).count() / 1000.0f;
                            if (elapsedAvoid > 0.4f) {
                                isAvoidingObstacle = false;
                                direction = 0;
                            }
                        }
                    
                        // Temporized random curve
                        if (!isAvoidingObstacle) {
                            float elapsedRandom = std::chrono::duration_cast<std::chrono::seconds>(now - lastTurnTime).count();
                    
                            if (isTurningRandomly) {
                                float turningTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - turnStartTime).count() / 1000.0f;
                                if (turningTime > random_turn_duration) {
                                    isTurningRandomly = false;
                                    direction = 0;
                                }
                            } else {
                                if (elapsedRandom > random_turn_time) {
                                    direction = (rand() % 2 == 0) ? -1 : 1;
                                    lastTurnTime = now;
                                    turnStartTime = now;
                                    isTurningRandomly = true;
                                    std::cout << "[Random Turn] --> direction = " << direction << std::endl;
                                }
                            }
                        }
                    
                        // Send commands
                        std::pair<float, float> drivevalues = {leftThrust, rightThrust};
                        cluon::data::TimeStamp sampleTime = cluon::time::now();
                    
                        opendlv::proxy::PedalPositionRequest pprl;
                        pprl.position(drivevalues.first);
                        od4.send(pprl, sampleTime, 0);
                    
                        opendlv::proxy::PedalPositionRequest pprr;
                        pprr.position(drivevalues.second);
                        od4.send(pprr, sampleTime, 1);
                    
                        // DEBUG
                        std::cout << "[DEBUG] direction=" << direction
                                  << " | Random=" << isTurningRandomly
                                  << " | Avoiding=" << isAvoidingObstacle
                                  << " | frontCount=" << frontStableCount
                                  << " | leftCount=" << leftStableCount
                                  << " | rightCount=" << rightStableCount << std::endl;
                    
                        break;
                    }
                    
                    
                    
                    case RobotState::TARGET_DETECTED: {
                        float imageCenterX = img.cols / 2.0f;
                        float error = targetPosition.x - imageCenterX;
                        //float kp = 0.003f;
                        float steering = kp * error;
                        steering = std::clamp(steering, -0.05f, 0.05f);

                        //float base_thrust = 0.08f;
                        leftThrust = base_thrust + steering;
                        rightThrust = base_thrust - steering;

                        opendlv::proxy::PedalPositionRequest pprl;
                        pprl.position(leftThrust);
                        od4.send(pprl, cluon::time::now(), 0);
                        opendlv::proxy::PedalPositionRequest pprr;
                        pprr.position(rightThrust);
                        od4.send(pprr, cluon::time::now(), 1);

                        float dy = std::abs(targetPosition.y - img.rows);

                        std::cout << "distance: " << dy << std::endl;

                        if (dy < stopThreshold) {
                            state = RobotState::STOPPED;
                        }
                        break;
                    }

                    case RobotState::STOPPED: {
                        opendlv::proxy::PedalPositionRequest pprl;
                        pprl.position(0.0f);
                        od4.send(pprl, cluon::time::now(), 0);
                        opendlv::proxy::PedalPositionRequest pprr;
                        pprr.position(0.0f);
                        od4.send(pprr, cluon::time::now(), 1);
                        break;
                    }
                }

                std::lock_guard<std::mutex> lck(distancesMutex);
                std::cout << "[STATE]: " << stateToString(state) << std::endl;
                std::cout << "front: " << front << ", back: " << rear << ", left: " << left << ", right: " << right <<std::endl;
                std::cout << "left thrust: " << leftThrust << ", right thrust: " << rightThrust << std::endl;

                if (verbose) {
                    cv::circle(img, targetPosition, 5, cv::Scalar(255, 0, 0), -1);
                    cv::line(img, cv::Point(0, maskStart), cv::Point(img.cols, maskStart), cv::Scalar(0, 255, 255), 3);
                    cv::imshow("Task1 FSM", img);
                    cv::imshow("Blue Target", blueMask);
                    cv::waitKey(1);
                }

                
            }
            retCode = 0;
        }
    }
    return retCode;
}
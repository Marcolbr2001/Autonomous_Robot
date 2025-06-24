#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <cstdlib> // For rand()
#include <ctime>   // For time()

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

enum class RobotState
{
    EXPLORING,
    TARGET_DETECTED,
    STOPPED,
    MARK_TARGET,
    RETURN_HOME
};

RobotState state = RobotState::EXPLORING;

const char *stateToString(RobotState stato)
{
    switch (stato)
    {
    case RobotState::EXPLORING:
        return "EXPLORING";
    case RobotState::TARGET_DETECTED:
        return "TARGET_DETECTED";
    case RobotState::STOPPED:
        return "STOPPED (At Green Target)";
    case RobotState::MARK_TARGET:
        return "MARK_TARGET";
    case RobotState::RETURN_HOME:
        return "RETURN_HOME";
    default:
        return "UNKNOWN";
    }
}

int32_t main(int32_t argc, char **argv)
{
    // Initialize random seed
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    int32_t retCode{1};
    auto cmd = cluon::getCommandlineArguments(argc, argv);

    if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
        (0 == cmd.count("width")) || (0 == cmd.count("height")))
    {
        std::cout << argv[0] << " attaches to a shared memory area containing an image." << std::endl;
        std::cout << "Usage:  " << argv[0] << " --cid=<OD4 session> --name=<shared memory> --width=<width> --height=<height> [optional_params]" << std::endl;
        return 1;
    }

    std::string const name{cmd["name"]};
    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};
    bool const verbose{cmd.count("verbose") != 0};

    float const base_thrust{(cmd["base-thrust"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust"])) : 0.08f};
    float const kp{(cmd["kp"].size() != 0) ? static_cast<float>(std::stof(cmd["kp"])) : 0.003f};

    float const frontThreshold{(cmd["front-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["front-threshold"])) : 0.30f};
    float const leftThreshold{(cmd["left-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["left-threshold"])) : 0.25f};
    float const rightThreshold{(cmd["right-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["right-threshold"])) : 0.25f};

    float const area_min{(cmd["area_min"].size() != 0) ? static_cast<float>(std::stof(cmd["area_min"])) : 200.0f};
    float const area_max{(cmd["area_max"].size() != 0) ? static_cast<float>(std::stof(cmd["area_max"])) : 70000.0f};
    float const stopThreshold{(cmd["stop-threshold"].size() != 0) ? static_cast<float>(std::stof(cmd["stop-threshold"])) : 40.0f};

    float const ignorePercentage{(cmd["ignore-top"].size() != 0) ? std::stof(cmd["ignore-top"]) : 0.5f};

    // HSV for Blue (Home Base)
    uint32_t const H_min{(cmd["H_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["H_min"])) : 100};
    uint32_t const S_min{(cmd["S_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["S_min"])) : 150};
    uint32_t const V_min{(cmd["V_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["V_min"])) : 50};
    uint32_t const H_max{(cmd["H_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["H_max"])) : 140};
    uint32_t const S_max{(cmd["S_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["S_max"])) : 255};
    uint32_t const V_max{(cmd["V_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["V_max"])) : 255};

    // HSV for Green (Target)
    uint32_t const GH_min = (cmd["GH_min"].size()) ? std::stoi(cmd["GH_min"]) : 40;
    uint32_t const GS_min = (cmd["GS_min"].size()) ? std::stoi(cmd["GS_min"]) : 50;
    uint32_t const GV_min = (cmd["GV_min"].size()) ? std::stoi(cmd["GV_min"]) : 50;
    uint32_t const GH_max = (cmd["GH_max"].size()) ? std::stoi(cmd["GH_max"]) : 80;
    uint32_t const GS_max = (cmd["GS_max"].size()) ? std::stoi(cmd["GS_max"]) : 255;
    uint32_t const GV_max = (cmd["GV_max"].size()) ? std::stoi(cmd["GV_max"]) : 255;

    // Thrust values for turning during exploration
    float const base_thrust_right{(cmd["base-thrust-right"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust-right"])) : 0.15f};
    float const base_thrust_left{(cmd["base-thrust-left"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust-left"])) : 0.15f};
    float const thrust_turn_front_left{(cmd["thrust-turn-front-left"].size() != 0) ? static_cast<float>(std::stof(cmd["thrust-turn-front-left"])) : 0.12f};
    float const thrust_turn_front_right{(cmd["thrust-turn-front-right"].size() != 0) ? static_cast<float>(std::stof(cmd["thrust-turn-front-right"])) : -0.05f};
    float const thrust_turn_left_left{(cmd["thrust-turn-left-left"].size() != 0) ? static_cast<float>(std::stof(cmd["thrust-turn-left-left"])) : 0.0f};
    float const thrust_turn_left_right{(cmd["thrust-turn-left-right"].size() != 0) ? static_cast<float>(std::stof(cmd["thrust-turn-left-right"])) : 0.12f};
    float const thrust_turn_right_left{(cmd["thrust-turn-right-left"].size() != 0) ? static_cast<float>(std::stof(cmd["thrust-turn-right-left"])) : 0.12f};
    float const thrust_turn_right_right{(cmd["thrust-turn-right-right"].size() != 0) ? static_cast<float>(std::stof(cmd["thrust-turn-right-right"])) : 0.0f};
    uint32_t const random_turn_time{(cmd["rtt"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["rtt"])) : 10};
    float const base_turn_duration{(cmd["base-turn-duration"].size() != 0) ? static_cast<float>(std::stof(cmd["base-turn-duration"])) : 7.0f};
    float const random_turn_coeff{(cmd["random-turn-coeff"].size() != 0) ? static_cast<float>(std::stof(cmd["random-turn-coeff"])) : 0.5f}; //+ or - the value to the base turn duration

    // Value for the stop state
    uint32_t const time_stop{(cmd["time_stop"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["time_stop"])) : 8};


    std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{name}};
    if (sharedMemory && sharedMemory->valid())
    {
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

        std::mutex distancesMutex;
        float distFront{0}, distRear{0}, distLeft{0}, distRight{0};
        auto onDistance = [&](cluon::data::Envelope &&env)
        {
            auto dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
            std::lock_guard<std::mutex> lck(distancesMutex);
            switch (env.senderStamp())
            {
            case 0:
                distFront = dr.distance();
                break;
            case 1:
                distLeft = dr.distance();
                break;
            case 3:
                distRear = dr.distance();
                break;
            case 5:
                distRight = dr.distance();
                break;
            }
        };
        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

        cv::Point2f targetPos(-1, -1);
        cv::Point2f homePos(-1, -1);

        float leftThrust = 0.0f;
        float rightThrust = 0.0f;

        const int RETURN_SEC = 60;
        auto lastReturnTime = std::chrono::steady_clock::now();

        auto lastRandomTurnTime = std::chrono::steady_clock::now();
        int lastTurnDirection = 0;

        float random_turn_duration = -1.0f;

        std::chrono::time_point<std::chrono::steady_clock> StartMarking;
        bool stopped = false;

        while (od4.isRunning())
        {
            cv::Mat img, imgHSV, blueMask, greenMask;

            sharedMemory->wait();
            sharedMemory->lock();
            void *raw = sharedMemory->data();
            if (raw != nullptr)
            {
                cv::Mat wrapped(height, width, CV_8UC3, reinterpret_cast<unsigned char *>(raw));
                img = wrapped.clone();
            }
            sharedMemory->unlock();

            if (img.empty())
                continue;

            cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

            // Create masks
            cv::inRange(imgHSV, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), blueMask);
            cv::inRange(imgHSV, cv::Scalar(GH_min, GS_min, GV_min), cv::Scalar(GH_max, GS_max, GV_max), greenMask);

            // Apply ignore percentage
            int maskStartRow = static_cast<int>(ignorePercentage * img.rows);
            if (maskStartRow > 0 && maskStartRow < img.rows)
            {
                cv::Mat roiForMask = cv::Mat::zeros(img.size(), CV_8UC1);
                roiForMask(cv::Rect(0, maskStartRow, img.cols, img.rows - maskStartRow)) = 1;
                blueMask.setTo(0, 1 - roiForMask);
                greenMask.setTo(0, 1 - roiForMask);
            }

            // Update Home Position if Visible
            if (state == RobotState::EXPLORING || state == RobotState::TARGET_DETECTED)
            {
                std::vector<std::vector<cv::Point>> blueCnts;
                cv::findContours(blueMask, blueCnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                if (!blueCnts.empty())
                {
                    auto &largestBlue = *std::max_element(blueCnts.begin(), blueCnts.end(),
                                                          [](const auto &a, const auto &b)
                                                          { return cv::contourArea(a) < cv::contourArea(b); });
                    double blueArea = cv::contourArea(largestBlue);
                    if (blueArea > area_min)
                    {
                        cv::Moments m = cv::moments(largestBlue);
                        if (std::abs(m.m00) > 1e-6)
                        {
                            homePos = {static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00)};
                            if (verbose)
                                std::cout << "Home base detected/updated at: (" << homePos.x << ", " << homePos.y << ")" << std::endl;
                        }
                    }
                }
            }

            // Main Timer Check
            auto currentTime = std::chrono::steady_clock::now();
            int elapsedSeconds = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastReturnTime).count());

            if(stopped){
                state = RobotState::STOPPED;
            }
            else if (elapsedSeconds >= RETURN_SEC &&
                (state == RobotState::EXPLORING || state == RobotState::TARGET_DETECTED))
            {
                if (verbose)
                    std::cout << "Timer (" << RETURN_SEC << "s) expired! Forcing RETURN_HOME." << std::endl;
                state = RobotState::RETURN_HOME;
            }

            // State Machine
            switch (state)
            {
            case RobotState::EXPLORING:
            {
                // Look for Green Target
                std::vector<std::vector<cv::Point>> greenCnts;
                cv::findContours(greenMask, greenCnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                bool greenFound = false;

                if (!greenCnts.empty())
                {
                    auto &largestGreen = *std::max_element(greenCnts.begin(), greenCnts.end(),
                                                           [](const auto &a, const auto &b)
                                                           { return cv::contourArea(a) < cv::contourArea(b); });
                    double greenArea = cv::contourArea(largestGreen);
                    if (greenArea > area_min && greenArea < area_max)
                    {
                        cv::Moments m = cv::moments(largestGreen);
                        if (std::abs(m.m00) > 1e-6)
                        {
                            targetPos = {static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00)};
                            state = RobotState::TARGET_DETECTED;
                            greenFound = true;
                            if (verbose)
                                std::cout << "Green target found at: (" << targetPos.x << ", " << targetPos.y << ")" << std::endl;
                        }
                    }
                }

                // If no target found, explore
                if (!greenFound)
                {
                    float currentDistFront = 0, currentDistLeft = 0, currentDistRight = 0;
                    {
                        std::lock_guard<std::mutex> lck(distancesMutex);
                        currentDistFront = distFront;
                        currentDistLeft = distLeft;
                        currentDistRight = distRight;
                    }

                    if (currentDistFront < frontThreshold && currentDistFront > 0.001f)
                    {
                        leftThrust = thrust_turn_front_left;
                        rightThrust = thrust_turn_front_right;
                        lastTurnDirection = 1;
                        lastRandomTurnTime = currentTime;
                    }
                    else if (currentDistLeft < leftThreshold && currentDistLeft > 0.001f)
                    {
                        leftThrust = thrust_turn_right_left;
                        rightThrust = thrust_turn_right_right;
                        lastTurnDirection = 1;
                        lastRandomTurnTime = currentTime;
                    }
                    else if (currentDistRight < rightThreshold && currentDistRight > 0.001f)
                    {
                        leftThrust = thrust_turn_left_left;
                        rightThrust = thrust_turn_left_right;
                        lastTurnDirection = -1;
                        lastRandomTurnTime = currentTime;
                    }
                    else
                    {
                        auto timeSinceLastRandomTurn = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastRandomTurnTime).count();
                        if (timeSinceLastRandomTurn > random_turn_time)
                        {
                            lastTurnDirection = (std::rand() % 2 == 0) ? -1 : 1;
                            if (lastTurnDirection == -1)
                            {
                                leftThrust = thrust_turn_left_left;
                                rightThrust = thrust_turn_left_right;
                                random_turn_duration = base_turn_duration + (rand()%int(random_turn_coeff*200))/(random_turn_coeff*200)*2-1;
                            }
                            else
                            {
                                leftThrust = thrust_turn_right_left;
                                rightThrust = thrust_turn_right_right;
                                random_turn_duration = base_turn_duration + (rand()%int(random_turn_coeff*200))/(random_turn_coeff*200)*2-1;
                            }
                            lastRandomTurnTime = currentTime;
                        }
                        else if (timeSinceLastRandomTurn > random_turn_duration)
                        {
                            leftThrust = base_thrust_left;
                            rightThrust = base_thrust_right;
                        }
                    }

                    opendlv::proxy::PedalPositionRequest pprl;
                    pprl.position(leftThrust);
                    od4.send(pprl, cluon::time::now(), 0);

                    opendlv::proxy::PedalPositionRequest pprr;
                    pprr.position(rightThrust);
                    od4.send(pprr, cluon::time::now(), 1);
                }
                break;
            }

            case RobotState::TARGET_DETECTED:
            {
                std::vector<std::vector<cv::Point>> greenCnts;
                cv::findContours(greenMask, greenCnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                bool greenFound = false;
                targetPos = cv::Point2f(-1, -1);

                if (!greenCnts.empty())
                {
                    auto &largestGreen = *std::max_element(greenCnts.begin(), greenCnts.end(),
                                                           [](const auto &a, const auto &b)
                                                           { return cv::contourArea(a) < cv::contourArea(b); });
                    double greenArea = cv::contourArea(largestGreen);
                    if (greenArea > area_min && greenArea < area_max)
                    {
                        cv::Moments m = cv::moments(largestGreen);
                        if (std::abs(m.m00) > 1e-6)
                        {
                            targetPos = {static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00)};
                            state = RobotState::TARGET_DETECTED;
                            greenFound = true;
                            if (verbose)
                                std::cout << "Green target found at: (" << targetPos.x << ", " << targetPos.y << ")" << std::endl;
                        }
                    }
                }
                if(!greenFound) {
                    if (targetPos.x < 0)
                    {
                        state = RobotState::EXPLORING;
                        break;
                    }
                } else {

                    float imageCenterX = img.cols / 2.0f;
                    float error = targetPos.x - imageCenterX;
                    float steering = std::max(-0.05f, std::min(0.05f, kp * error));

                    leftThrust = base_thrust + steering;
                    rightThrust = base_thrust - steering;

                }

                    opendlv::proxy::PedalPositionRequest pprl;
                    pprl.position(leftThrust);
                    od4.send(pprl, cluon::time::now(), 0);

                    opendlv::proxy::PedalPositionRequest pprr;
                    pprr.position(rightThrust);
                    od4.send(pprr, cluon::time::now(), 1);

                    float dyToTarget = std::abs(targetPos.y - (img.rows - 1));
                    if (verbose)
                        std::cout << "TARGET_DETECTED: Navigating. Target Y: " << targetPos.y << ", dyToTarget: " << dyToTarget << std::endl;

                    if ((dyToTarget < stopThreshold) && (std::chrono::duration_cast<std::chrono::seconds>(currentTime - StartMarking).count() >= time_stop + 1 ))
                    {
                        if (verbose)
                            std::cout << "Arrived at green target." << std::endl;
                        state = RobotState::STOPPED;
                    }
                break;
            }

            case RobotState::STOPPED:
            {
                if (verbose)
                    std::cout << "State STOPPED: Halting at green target." << std::endl;

                if(!stopped){
                    StartMarking = currentTime;
                    stopped= true;
                }

                auto timeMarking = std::chrono::duration_cast<std::chrono::seconds>(currentTime - StartMarking).count();

                currentTime = std::chrono::steady_clock::now();
                timeMarking = std::chrono::duration_cast<std::chrono::seconds>(currentTime - StartMarking).count();
                if(timeMarking < 2){
                    leftThrust = base_thrust_left;
                    rightThrust = base_thrust_right;
                }
                else if(((timeMarking-2) < float(time_stop)/4) || ((timeMarking-2) > float(time_stop)/2 && ((timeMarking-2) < float(time_stop)*3/4))){
                    leftThrust = thrust_turn_left_right;
                    rightThrust = -thrust_turn_left_right;
                }
                else {
                    leftThrust = -thrust_turn_right_left;
                    rightThrust = thrust_turn_right_left;
                }
                opendlv::proxy::PedalPositionRequest pprl;
                pprl.position(leftThrust);
                od4.send(pprl, cluon::time::now(), 0);

                opendlv::proxy::PedalPositionRequest pprr;
                pprr.position(rightThrust);
                od4.send(pprr, cluon::time::now(), 1);
                
                if(timeMarking >= time_stop){
                    stopped = false;
                    state = RobotState::MARK_TARGET;
                }
                break;
            }

            case RobotState::MARK_TARGET:
            {
                if (verbose)
                    std::cout << "State MARK_TARGET: 'Marking' green target." << std::endl;

                targetPos = cv::Point2f(-1, -1);

                if (elapsedSeconds >= RETURN_SEC)
                {
                    if (verbose)
                        std::cout << "MARK_TARGET: Time up, transitioning to RETURN_HOME." << std::endl;
                    state = RobotState::RETURN_HOME;
                }
                else
                {
                    if (verbose)
                        std::cout << "MARK_TARGET: Time remaining, transitioning to EXPLORING." << std::endl;
                    state = RobotState::EXPLORING;
                }
                break;
            }

            case RobotState::RETURN_HOME:
            {
                cv::Point2f currentViewHomePos(-1, -1);
                homePos = cv::Point2f(-1, -1);

                // Try to see home in current view
                std::vector<std::vector<cv::Point>> blueCntsReturn;
                cv::findContours(blueMask, blueCntsReturn, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                if (!blueCntsReturn.empty())
                {
                    auto &largestBlueReturn = *std::max_element(blueCntsReturn.begin(), blueCntsReturn.end(),
                                                                [](const auto &a, const auto &b)
                                                                { return cv::contourArea(a) < cv::contourArea(b); });
                    double blueAreaReturn = cv::contourArea(largestBlueReturn);
                    if (blueAreaReturn > area_min)
                    {
                        cv::Moments m = cv::moments(largestBlueReturn);
                        if (std::abs(m.m00) > 1e-6)
                        {
                            currentViewHomePos = {static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00)};
                            homePos = currentViewHomePos;
                            if (verbose)
                                std::cout << "RETURN_HOME: Home re-acquired in view at (" << homePos.x << ", " << homePos.y << ")" << std::endl;
                        }
                    }
                }
                if (homePos.x > 0 && homePos.y > 0)
                {
                    float imageCenterX = img.cols / 2.0f;
                    float error = homePos.x - imageCenterX;
                    float steering = std::max(-0.05f, std::min(0.05f, kp * error));

                    leftThrust = base_thrust + steering;
                    rightThrust = base_thrust - steering;

                    float dyToHome = std::abs(homePos.y - (img.rows - 1));
                    if (verbose)
                        std::cout << "RETURN_HOME: Navigating to home. Home Y: " << homePos.y << ", dyToHome: " << dyToHome << std::endl;

                    if (dyToHome < stopThreshold)
                    {
                        if (verbose)
                            std::cout << "Arrived at home base!" << std::endl;
                        lastReturnTime = std::chrono::steady_clock::now();
                        state = RobotState::EXPLORING;
                    }
                }
                else
                {
                    if (verbose)
                        std::cout << "RETURN_HOME: Home base position unknown. Spinning to find blue." << std::endl;
                    leftThrust = thrust_turn_left_left;
                    rightThrust = thrust_turn_left_right;
                }
            }

                opendlv::proxy::PedalPositionRequest pprl;
                pprl.position(leftThrust);
                od4.send(pprl, cluon::time::now(), 0);

                opendlv::proxy::PedalPositionRequest pprr;
                pprr.position(rightThrust);
                od4.send(pprr, cluon::time::now(), 1);
                break;
            }

            if (verbose)
            {
                // Draw target and home positions on the image
                if (targetPos.x >= 0)
                    cv::circle(img, targetPos, 7, cv::Scalar(0, 255, 0), 2);
                if (homePos.x >= 0)
                    cv::circle(img, homePos, 7, cv::Scalar(255, 0, 0), 2);

                // Draw the line for the ignored top part
                int maskStartRow2 = static_cast<int>(ignorePercentage * img.rows);
                cv::line(img, cv::Point(0, maskStartRow2), cv::Point(img.cols, maskStartRow2), cv::Scalar(0, 255, 255), 1);

                cv::imshow("Robot View", img);
                if (!blueMask.empty())
                    cv::imshow("Blue Mask (Home)", blueMask);
                if (!greenMask.empty())
                    cv::imshow("Green Mask (Target)", greenMask);

                std::cout << "[State: " << stateToString(state) << "] ";
                std::cout << "Timer: " << elapsedSeconds << "/" << RETURN_SEC << "s | ";
                std::cout << "LThrust: " << std::fixed << std::setprecision(3) << leftThrust
                        << ", RThrust: " << rightThrust << " | ";
                {
                    std::lock_guard<std::mutex> lck(distancesMutex);
                    std::cout << "Dist F:" << distFront << " L:" << distLeft << " R:" << distRight;
                }
                std::cout << std::endl;
                cv::waitKey(1);
            }
        }
    } 
    else
    {
        std::cerr << "Failed to open shared memory: " << name << std::endl;
    }
    return retCode;
}

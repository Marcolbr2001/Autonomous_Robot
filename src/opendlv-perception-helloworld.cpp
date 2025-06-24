/*
* Copyright (C) 2024 OpenDLV
* 
* Version with improved control logic:
* - Uses the average of the first 3 visible cone pairs (if available).
* - Falls back to left or right cone wall if only one color is visible.
* - Falls back to last known steering direction if no cones are visible.
* - Includes simple PID controller for smoother motion.
* - Perception remains unchanged.
*/

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <cmath> // Needed for M_PI
#include <opencv2/calib3d.hpp> // Required for cv::projectPoints

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"


// This function calculates the centroids (geometric centers) of contours and returns them sorted from bottom to top in the image.
std::vector<cv::Point2f> getSortedCentroids(const std::vector<std::vector<cv::Point>>& contours) 
{
  std::vector<cv::Point2f> centroids;
  for (const auto& cnt : contours) 
  {
    if (cv::contourArea(cnt) > 100) { // Ignore tiny noise contours
      cv::Moments m = cv::moments(cnt);
      if (m.m00 > 0) 
      {
        centroids.emplace_back(m.m10 / m.m00, m.m01 / m.m00);
      }
    }
  }
  // Sort centroids from bottom (close) to top (far)
  std::sort(centroids.begin(), centroids.end(), [](const cv::Point2f& a, const cv::Point2f& b) 
  {
    return a.y > b.y;
  });

  return centroids;
}

// Function that computes trajectory
cv::Point2f cubic_bezier_4p(const cv::Point2f& p0, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, double t) {
  double one_minus_t = 1.0 - t;
  double x = std::pow(one_minus_t, 3) * p0.x +
             3 * std::pow(one_minus_t, 2) * t * p1.x +
             3 * one_minus_t * std::pow(t, 2) * p2.x +
             std::pow(t, 3) * p3.x;
  double y = std::pow(one_minus_t, 3) * p0.y +
             3 * std::pow(one_minus_t, 2) * t * p1.y +
             3 * one_minus_t * std::pow(t, 2) * p2.y +
             std::pow(t, 3) * p3.y;
  cv::Point2f p;
  p.x=float(x);
  p.y=float(y);
  return p;
}


int32_t main(int32_t argc, char **argv) {
    
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);

  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
  (0 == cmd.count("width")) || (0 == cmd.count("height"))) 
  {
    std::cout << argv[0] << " attaches to a shared memory area containing an image." << std::endl;
    std::cout << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> --width=<width> --height=<height> [--verbose]" << std::endl;
  } 
  else {
    std::string const name{cmd["name"]};
    uint32_t const width{(cmd["width"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["width"])) : 1280};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};
    
    float const base_thrust{(cmd["base-thrust"].size() != 0) ? static_cast<float>(std::stof(cmd["base-thrust"])) : 0.1f};
    float const ki{(cmd["ki"].size() != 0) ? static_cast<float>(std::stof(cmd["ki"])) : 0.001f};
    float const kp{(cmd["kp"].size() != 0) ? static_cast<float>(std::stof(cmd["kp"])) : 0.03f};
    
    float const offset{(cmd["offset"].size() != 0) ? static_cast<float>(std::stof(cmd["offset"])) : 0.1f};
    float const recover_left{(cmd["recover-left"].size() != 0) ? static_cast<float>(std::stof(cmd["recover-left"])) : 0.1f};
    float const recover_right{(cmd["recover-right"].size() != 0) ? static_cast<float>(std::stof(cmd["recover-right"])) : 0.15f};

    uint32_t const bH_min{(cmd["bH_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["bH_min"])) : 100};
    uint32_t const bS_min{(cmd["bS_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["bS_min"])) : 100};
    uint32_t const bV_min{(cmd["bV_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["bV_min"])) : 50};
    uint32_t const bH_max{(cmd["bH_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["bH_max"])) : 140};
    uint32_t const bS_max{(cmd["bS_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["bS_max"])) : 255};
    uint32_t const bV_max{(cmd["bV_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["bV_max"])) : 255};

    uint32_t const yH_min{(cmd["yH_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yH_min"])) : 20};
    uint32_t const yS_min{(cmd["yS_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yS_min"])) : 130};
    uint32_t const yV_min{(cmd["yV_min"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yV_min"])) : 130};
    uint32_t const yH_max{(cmd["yH_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yH_max"])) : 35};
    uint32_t const yS_max{(cmd["yS_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yS_max"])) : 255};
    uint32_t const yV_max{(cmd["yV_max"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yV_max"])) : 255};

    float const maskFraction = (cmd["mask-threshold"].size() != 0) ? std::stof(cmd["mask-threshold"]) : 0.5f;
    float const flatten = (cmd["flatten"].size() != 0) ? std::stof(cmd["flatten"]) : 2.5f;
    uint32_t const yOffset{(cmd["yOffset"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["yOffset"])) : 50};
    uint32_t const min_cone_area{(cmd["min-cone-area"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["min-cone-area"])) : 150};


    uint32_t const traj_size{(cmd["traj-size"].size() != 0) ? static_cast<uint32_t>(std::stoi(cmd["traj-size"])) : 65};

    bool const verbose{cmd.count("verbose") != 0};

    std::unique_ptr<cluon::SharedMemory> sharedMemory{ new cluon::SharedMemory{name} };
    if (sharedMemory && sharedMemory->valid()) {
      size_t expectedSize = width * height * 3;
      
      if (sharedMemory->size() < expectedSize) 
      {
        std::cerr << "Error: shared memory too small." << std::endl;
        return 1;
      }

      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

      std::mutex distancesMutex;
      float front{0}, rear{0}, left{0}, right{0};
      auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env) {
        auto senderStamp = env.senderStamp();
        opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
        std::lock_guard<std::mutex> lck(distancesMutex);
        switch (senderStamp) 
        {
          case 0: front = dr.distance(); break;
          case 2: rear = dr.distance(); break;
          case 1: left = dr.distance(); break;
          case 3: right = dr.distance(); break;
        }
      };
      od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

      // PID controller state
      static float integral = 0.0f;

      while (od4.isRunning()) {
        cv::Mat img;

        // Read the image from shared memory
        sharedMemory->wait();
        sharedMemory->lock();
        
        void* raw = sharedMemory->data();
        if (raw != nullptr) 
        {
          cv::Mat wrapped(height, width, CV_8UC3, reinterpret_cast<unsigned char*>(raw));
          img = wrapped.clone(); // Make a local copy
        } 
        else 
        {
          std::cerr << "[ERROR] sharedMemory->data() is null!" << std::endl;
          sharedMemory->unlock();
          continue;
        }
        
        sharedMemory->unlock();

        if (img.empty()) 
        {
          std::cerr << "[ERROR] Invalid image, skipping frame." << std::endl;
          continue;
        }

        // ====================== PERCEPTION ====================== //

        // Posizione verticale del centro dell'ellisse
        float const ellipticalY = img.rows * (1.0f - maskFraction) + yOffset;  // es: 0.3f = più bassa

        // Maschera nera iniziale
        cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);

        // Centro e assi dell'ellisse
        cv::Point center(img.cols / 2, static_cast<int>(ellipticalY));
        cv::Size axes(img.cols, static_cast<int>(img.rows / flatten));

        // Disegna una ellisse piena bianca
        cv::ellipse(mask, center, axes, 0, 0, 360, cv::Scalar(255), -1);

        // Convert BGR to HSV to make color filtering easier
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // Threshold HSV image to extract blue and yellow cones
        cv::Scalar hsvBlueLow(bH_min, bS_min, bV_min), hsvBlueHigh(bH_max, bS_max, bV_max);
        cv::Scalar hsvYellowLow(yH_min, yS_min, yV_min), hsvYellowHigh(yH_max, yS_max, yV_max);

        cv::Mat blueMask, yellowMask;
        cv::inRange(hsv, hsvBlueLow, hsvBlueHigh, blueMask);
        cv::inRange(hsv, hsvYellowLow, hsvYellowHigh, yellowMask);

        // Apply the mask to ignore top part, but do NOT erase the image
        cv::bitwise_and(blueMask, mask, blueMask);
        cv::bitwise_and(yellowMask, mask, yellowMask);

        // Apply morphological operations to clean up
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
        cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, kernel);
        
        // Find contours
        std::vector<std::vector<cv::Point>> blueContours, yellowContours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(blueMask, blueContours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(yellowMask, yellowContours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


        // -- STEP: Filtraggio geometrico per contorni validi --
        std::vector<std::vector<cv::Point>> filteredBlueContours, filteredYellowContours;
        
        for (const auto& cnt : blueContours) {
            double area = cv::contourArea(cnt);
            if (area < min_cone_area || area > 3000) continue;
            cv::Rect bb = cv::boundingRect(cnt);
            double ar = static_cast<double>(bb.height) / static_cast<double>(bb.width);
            if (ar < 0.6 || ar > 3.5) continue;
            filteredBlueContours.push_back(cnt);
        }
        
        for (const auto& cnt : yellowContours) {
            double area = cv::contourArea(cnt);
            if (area < min_cone_area || area > 3000) continue;
            cv::Rect bb = cv::boundingRect(cnt);
            double ar = static_cast<double>(bb.height) / static_cast<double>(bb.width);
            if (ar < 0.6 || ar > 3.5) continue;
            filteredYellowContours.push_back(cnt);
        }
        
        // Get centroids
        std::vector<cv::Point2f> blueCentroids = getSortedCentroids(filteredBlueContours);
        std::vector<cv::Point2f> yellowCentroids = getSortedCentroids(filteredYellowContours);

        // For debug: merged mask
        cv::Mat combinedCones;
        cv::bitwise_or(blueMask, yellowMask, combinedCones);

        // ====================== PERCEPTION ends ====================== //



        // ====================== TRAJECTORY evaluation ====================== //

        // Midpoint declaration
        cv::Point2f midPoint(-1, -1); // Initialize with this default values
        std::vector<cv::Point2f> midpointsarr; //initialize where we store the midpoints
        bool validMidpoint = false;
        // Try to compute a stable midpoint using first 3 pairs
        size_t nPairs = std::min(blueCentroids.size(), yellowCentroids.size());
        if (nPairs >= 3) 
        {
          // Using the first 3 closest pairs gives a more robust path estimate
          cv::Point2f sumMidpoints(0, 0);
          for (size_t i = 0; i < 3; ++i)
          {
          sumMidpoints += 0.5f * (blueCentroids[i] + yellowCentroids[i])*(7.0f-i*3.0f); //first couple is weighted 7, second 4 and third 1
          midpointsarr.push_back(0.5f * (blueCentroids[i] + yellowCentroids[i]));
          }
          midPoint = sumMidpoints * (1.0f / 12.0f); //is a weghted mean, all weight add up to 12
          validMidpoint = true;

        }
        else if (!blueCentroids.empty() && !yellowCentroids.empty())
        {
          midPoint = 0.5f * (blueCentroids[0] + yellowCentroids[0]);
          validMidpoint = true;
        } 
        else if (!yellowCentroids.empty())
        {
          midPoint = yellowCentroids[0] + cv::Point2f(+400.0f, 0);
          validMidpoint = true;
        } 
        else if (!blueCentroids.empty())
        {
          midPoint = blueCentroids[0] + cv::Point2f(-400.0f, 0);
          validMidpoint = true;
        }
        
        std::vector<cv::Point2f> trajectory;
        int size_trajectory=0;
        if(midpointsarr.size()!=0){
          cv::Point2f p0(img.cols / 2.0f, img.rows);
          for (double t = 0.0; t <= 1.0; t += 0.01) {
            trajectory.push_back(cubic_bezier_4p(p0, midpointsarr[0], midpointsarr[1], midpointsarr[2], t));
            size_trajectory++;
          }
        }

        // ====================== TRAJECTORY ends ====================== //

        // ====================== CONTROLLER =========================== // 
        if (validMidpoint) {

          float centerX = img.cols / 2.0f;
          float error = 0.0;
          
          if (trajectory.size() > traj_size)
            error = trajectory[traj_size].x - centerX;//midPoint.x - centerX;
          else
            error = midPoint.x - centerX;

          // -------- PI -------- //
          //  kp = 0.03f, ki = 0.001f;
          integral += error;
          float steeringDeg = kp * error + ki * integral;

          //float steeringStraightDeg = 0.0f; // trial to go straight
          float steeringRad = steeringDeg / 180.0f * static_cast<float>(M_PI); // convert to radians
          steeringRad = std::clamp(steeringRad, -0.663f, 0.663f); // safety clamp for 38 deg max

          // -------------------- //

          // -- Sending commands -- //
          // static float base_thrust = 0.1f;
          
          float leftsteer = base_thrust+(offset+steeringRad/static_cast<float>(M_PI));
          float rightsteer = base_thrust+(offset-steeringRad/static_cast<float>(M_PI));

          std::pair<float,float> drivevalues = {leftsteer,rightsteer};
          cluon::data::TimeStamp sampleTime = cluon::time::now();

          opendlv::proxy::PedalPositionRequest pprl;
          pprl.position(drivevalues.first);
          od4.send(pprl, sampleTime, 0);

          opendlv::proxy::PedalPositionRequest pprr;
          pprr.position(drivevalues.second);
          od4.send(pprr, sampleTime, 1);

          std::cout << "[DEBUG] Steer = " << steeringRad << " rad (" << steeringDeg << " deg)" << std::endl; // Print debug commands
          std::cout << "[DEBUG] Left  = " << leftsteer << ",  Right = " << rightsteer << std::endl; // Print debug commands

          // ---------------------- //

        }
        else
        {
          integral = 0.0f;

          std::pair<float,float> drivevalues = {recover_left, recover_right};
          cluon::data::TimeStamp sampleTime = cluon::time::now();

          opendlv::proxy::PedalPositionRequest pprl;
          pprl.position(drivevalues.first);
          od4.send(pprl, sampleTime, 0);

          opendlv::proxy::PedalPositionRequest pprr;
          pprr.position(drivevalues.second);
          od4.send(pprr, sampleTime, 1);

          std::cout << "[WARN] No valid midpoint: turning left" << std::endl; // Print debug commands

        }
        // ===================== CONTROLLER ends =========================== //

        // ===================== IMAGE printing ============================ //
                
        if (verbose) {

          // --- Printing trajectory -- //
          float try_centerX = img.cols / 2.0f;
          cv::circle(img, cv::Point2f(try_centerX, img.rows - 5), 5, cv::Scalar(255, 0, 255), -1); // Print the robot center of vision in pink
          cv::Point2f robotBase(img.cols / 2.0f, img.rows);

          if(trajectory.size()!=0){
            for (int i=0;i<size_trajectory;i++){
              if(i==0){
                cv::circle(img, trajectory[i], 5, cv::Scalar(255, 0, 255), -1);
              } else if (i==size_trajectory-1){
                cv::circle(img, trajectory[i], 5, cv::Scalar(255, 0, 255), -1);
                cv::line(img, trajectory[i-1], trajectory[i], cv::Scalar(255, 255, 0), 2);
              } else{
                cv::line(img, trajectory[i-1], trajectory[i], cv::Scalar(255, 255, 0), 2);
              }
            }
            cv::circle(img, trajectory[traj_size], 5, cv::Scalar(0, 0, 255), -1);

          }
          else
          {
            cv::circle(img, midPoint, 5, cv::Scalar(0, 0, 255), -1); // Print the computed midPointCamera in red
            cv::line(img, robotBase, midPoint, cv::Scalar(255, 255, 0), 2); // Print a line that connect robot center to trajectory midpoind
          }
          // ------------------------- //

          // Draw yellow line for debug
          //cv::line(img, cv::Point(0, maskLineY), cv::Point(img.cols, maskLineY), cv::Scalar(0, 255, 255), 2);
          //cv::ellipse(img, center, axes, 0, 0, 180, cv::Scalar(0, 255, 255), 3); // SOLO per disegno
          cv::ellipse(img, center, axes, 0, 0, 360, cv::Scalar(0, 255, 255), 2);  // ellisse di contorno visibile in giallo

          // Draw contours in green for boith images
          // for (const auto& cnt : blueContours)
          // {
          //   if (cv::contourArea(cnt) > 100)
          //   {
          //     cv::drawContours(img, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
          //     cv::drawContours(combinedCones, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
          //   }
          // }

          for (const auto& cnt : filteredBlueContours)
          {
              // double area = cv::contourArea(cnt);
              // if (area < 100 || area > 3000) continue;
          
              // cv::Rect bb = cv::boundingRect(cnt);
              // double ar = static_cast<double>(bb.height) / static_cast<double>(bb.width);
              // if (ar < 0.6 || ar > 3.5) continue;  // filtro per forma (altezza/larghezza)
          
              cv::drawContours(img, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
              cv::drawContours(combinedCones, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
          }

          for (const auto& cnt : filteredYellowContours)
          {
              // double area = cv::contourArea(cnt);
              // if (area < 100 || area > 3000) continue;
          
              // cv::Rect bb = cv::boundingRect(cnt);
              // double ar = static_cast<double>(bb.height) / static_cast<double>(bb.width);
              // if (ar < 0.6 || ar > 3.5) continue;  // filtro per forma (altezza/larghezza)
          
              cv::drawContours(img, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
              cv::drawContours(combinedCones, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
          }
          
          
          if (blueCentroids.size() >= 3 && yellowCentroids.size() >= 3) {
            for (int i = 0; i < 3; ++i) {
                cv::circle(img, blueCentroids[i], 5, cv::Scalar(255, 255, 255), -1); // bianco
                cv::circle(img, yellowCentroids[i], 5, cv::Scalar(255, 255, 255), -1);
            }
        }else{
        
          
          for (const auto& c : blueCentroids) {
              cv::circle(img, c, 4, cv::Scalar(255, 0, 255), -1); // blue dot
          }
          for (const auto& c : yellowCentroids) {
              cv::circle(img, c, 4, cv::Scalar(255, 0, 255), -1); // yellow dot
          }
        }
        
          cv::imshow("Processed Image", img);

          cv::waitKey(1);
        }

        // ===================== IMAGE printing ends ======================== //


      }
    }
    retCode = 0;
  }
  return retCode;
}

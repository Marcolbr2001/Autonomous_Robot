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

//------------------------------------------------

const double fx     = 799, fy     = 704;
const double cx     = 640, cy     = 360;
// const double A_real = std::sqrt(3.0)/4.0 * 0.10 * 0.10;  // 正三角形コーン面積[m²]
const double pitch  =  0.0 * CV_PI/180.0;  // ピッチ30°下向き
const double roll   =  0.0 * CV_PI/180.0;  // ロール0°
cv::Mat Rx = (cv::Mat_<double>(3,3)<<
    1,            0,             0,
    0, std::cos(roll), -std::sin(roll),
    0, std::sin(roll),  std::cos(roll)
);
cv::Mat Ry = (cv::Mat_<double>(3,3)<<
    std::cos(pitch), 0, std::sin(pitch),
                0, 1,              0,
  -std::sin(pitch), 0, std::cos(pitch)
);
cv::Mat R = Rx * Ry;  // カメラ→ワールド回転行列

const double minA = 100.0, maxA = 3000.0;

//--------------------------------------------------

// This function calculates the centroids (geometric centers) of contours
// and returns them sorted from bottom to top in the image.
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

std::vector<cv::Point2f> processContours(const std::vector<std::vector<cv::Point>>& contours, const std::string label) {
  std::vector<cv::Mat> vectPwB;
  std::vector<cv::Point2f> vectxy;
  for (auto& cnt : contours) {
    double area_px = cv::contourArea(cnt);
    if (area_px < 100.0 || area_px > 2000.0) continue;

    // Projection on the camera image
    cv::Moments m = cv::moments(cnt);
    double u = m.m10 / m.m00;
    double v = m.m01 / m.m00 + 360;

    cv::Mat dirCam   = (cv::Mat_<double>(3,1) << -(u - cx)/fx, 1.0, -(v - cy)/fy);
    cv::Mat dirWorld = R * dirCam;
    const double camHeight = 0.095;
    double s = camHeight / -dirWorld.at<double>(2);
    cv::Mat PwB = dirWorld * s; 

    std::string test=label;
    // Print of values
    /*std::cout
      << "[" << label << "] B:( "
      << PwB.at<double>(0) << ", "
      << PwB.at<double>(1) << ", "
      << PwB.at<double>(2) + camHeight
      << " ) m\n";*/

    //returning just the x and y
    float x = PwB.at<float>(0,0);
    float y = PwB.at<float>(1,0);

    cv::Point2f xy;

    xy.x = x;
    xy.y = y;

    vectxy.push_back(xy);
  }

  return vectxy;
}

/*void filterContours(const std::vector<std::vector<cv::Point>>& in,
  std::vector<std::vector<cv::Point>>& out){
  for(auto &cnt : in)
  {
    double A = cv::contourArea(cnt);
    if(A < minA || A > maxA) continue;

    // 1) バウンディングボックスでアスペクト比チェック
    cv::Rect  bb = cv::boundingRect(cnt);
    double    ar = double(bb.height) / double(bb.width);
    if(ar > 3)           continue;
    if(ar < 0.3)           continue; // 縦長すぎないものだけ

    // 2) ソリディティ(面積/矩形面積)チェック
    double solidity = A / double(bb.area());
    if(solidity < 0.3)     continue; // 凸凹過ぎるものは落とす

    // 3) 近似ポリゴンで頂点数チェック（おおよそ三角形）
    // std::vector<cv::Point> approx;
    // cv::approxPolyDP(cnt, approx, 0.02 * cv::arcLength(cnt, true), true);
    // if(approx.size() != 3) continue; // ３頂点（三角形）だけ

    out.push_back(cnt);
    
  }
}*/

  void filterContours(const std::vector<std::vector<cv::Point>>& in,
    std::vector<std::vector<cv::Point>>& out, int imageHeight)
    {
      for (auto &cnt : in)
      {
      double A = cv::contourArea(cnt);
      if (A < minA || A > maxA) continue;
      cv::Rect bb = cv::boundingRect(cnt);
      // IGNORA LA PARTE ALTA DELL'IMMAGINE
      if (bb.y < imageHeight / 2) continue;
      double ar = double(bb.height) / double(bb.width);
      if (ar > 3) continue;
      if (ar < 0.3) continue;
      double solidity = A / double(bb.area());
      if (solidity < 0.3) continue;
      out.push_back(cnt);
      }
    }

cv::Point2d project3DPointBackToImage(const cv::Point3d& worldPoint_displacement)
{
// The input worldPoint_displacement is a vector from the camera center to the 3D point,
// expressed in the world coordinate system's orientation.
// To get the coordinates of the 3D point in the camera's own frame (relative to the camera origin),
// we rotate this vector using the R matrix.
cv::Mat world_disp_mat = (cv::Mat_<double>(3, 1) << worldPoint_displacement.x, worldPoint_displacement.y, worldPoint_displacement.z);
cv::Mat camera_point_mat = R * world_disp_mat; // This is the point in the camera frame

double Xc = camera_point_mat.at<double>(0, 0);
double Yc = camera_point_mat.at<double>(1, 0);
double Zc = camera_point_mat.at<double>(2, 0);

cv::Point2d imagePoint(-1, -1); // Initialize with invalid coordinates

// Project the point from the camera frame to the image plane
if (Zc > 0) // A point must have a positive Zc to be in front of the camera and project
{
double u = fx * (Xc / Zc) + cx;
double v = fy * (Yc / Zc) + cy;
imagePoint.x = u;
imagePoint.y = v;
}


return imagePoint;
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
    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};
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
        // Eliminating the upper part of the image
        /*int halfY = img.rows / 2;
        cv::Rect roiRect(0, halfY, img.cols, img.rows - halfY);
        cv::Mat img = img(roiRect).clone();
        img.copyTo(img(roiRect));*/

        /*cv::Mat img = cv::Mat::zeros(img.size(), CV_8UC1);
        cv::Rect roi(0, img.rows / 2, img.cols, img.rows / 2);
        img = img(roi);*/


        cv::Mat lab, bChan;
        cv::cvtColor(img, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> labChs;
        cv::split(lab, labChs);
        // OpenCV では Lab の b* が 0…255 にスケーリングされている
        bChan = labChs[2];  // b* チャネル

        std::vector<uchar> vals;
        vals.reserve(bChan.total());
        for (int y = 0; y < bChan.rows; ++y) {
          for (int x = 0; x < bChan.cols; ++x) {
            vals.push_back(bChan.at<uchar>(y, x));
          }
        }
        std::sort(vals.begin(), vals.end());
        int N = static_cast<int>(vals.size());
        uchar thBlue  = vals[ N *  2 / 100 ];  // 下位 2%
        uchar thYellow= vals[ N * 98 / 100 ];  // 上位 2%

        // --- 3) マスク生成 ---
        cv::Mat blueCones, yellowCones, blueClose, blueOpen, yellowClose, yellowOpen;
        cv::threshold(bChan, blueCones,  thBlue,   255, cv::THRESH_BINARY_INV);
        cv::threshold(bChan, yellowCones, thYellow,255, cv::THRESH_BINARY);

        // 以降、maskBlue/maskYellow に morphologyEx → findContours → processContours など

        // === 前処理 (平均値フィルタ → Opening → Closing) ===
        cv::morphologyEx(blueCones,  blueOpen,  cv::MORPH_OPEN,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));
        cv::morphologyEx(blueOpen,  blueClose, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(16,16)));
        //cv::imshow("blue cones", blueCones);
        //cv::imshow("Processed Blue Mask", blueClose);

        // cv::imshow("yellow cones", yellowCones);
        cv::morphologyEx(yellowCones, yellowOpen,  cv::MORPH_OPEN,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));
        cv::morphologyEx(yellowOpen, yellowClose, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(16,16)));

        // Find contours for both cone colors
        std::vector<std::vector<cv::Point>> contoursBlue, contoursYellow;
        cv::findContours(blueClose,   contoursBlue,   cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(yellowClose, contoursYellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        std::vector<std::vector<cv::Point>> goodBlue, goodYellow;

        filterContours(contoursBlue,  goodBlue, img.rows);
        filterContours(contoursYellow,goodYellow, img.rows);

        std::vector<cv::Point2f> YellowPosition,BluePosition;
        BluePosition = processContours(contoursBlue, "BlueCone");
        YellowPosition = processContours(contoursYellow,"YellowCone");

        // ① 輪郭ベクトル goodBlue/goodYellow をバイナリマスクに描画
        cv::Mat maskBlue = cv::Mat::zeros(blueClose.size(), CV_8UC1);
        cv::Mat maskYellow = cv::Mat::zeros(blueClose.size(), CV_8UC1);
        for(auto &cnt : goodBlue) {
            cv::drawContours(maskBlue, std::vector<std::vector<cv::Point>>{cnt},
                            0, cv::Scalar(255), cv::FILLED);
        }
        for(auto &cnt : goodYellow) {
            cv::drawContours(maskYellow, std::vector<std::vector<cv::Point>>{cnt},
                            0, cv::Scalar(255), cv::FILLED);
        }

        // ② マスクをカラー化
        cv::Mat colorMask = cv::Mat::zeros(maskBlue.size(), CV_8UC3);
        cv::Mat tmp;
        cv::cvtColor(maskBlue, tmp, cv::COLOR_GRAY2BGR);
        colorMask.setTo(cv::Scalar(255,0,0), maskBlue);      // 青領域を青色で
        colorMask.setTo(cv::Scalar(0,255,255), maskYellow);  // 黄領域を黄色で

        // ③ 元画像（img）と同サイズなので安心して合成できる
        cv::Mat blended;
        cv::addWeighted(img, 0.7, colorMask, 0.3, 0.0, blended);
        //cv::imshow("Blue & Yellow Overlay", blended);

        // Get centroids of the contours, sorted from bottom to top
        std::vector<cv::Point2f> blueCentroids = getSortedCentroids(contoursBlue);
        std::vector<cv::Point2f> yellowCentroids = getSortedCentroids(contoursYellow);

        // Merge binary images for visual debug window
        cv::Mat combinedCones;
        cv::bitwise_or(maskBlue, maskYellow, combinedCones);
        // ====================== PERCEPTION ends ====================== //


        // ====================== TRAJECTORY evaluation ====================== //
        // Midpoint declaration
        cv::Point2f midPoint(-1, -1); // Initialize with this default values
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

        //Try to compute a stable midpoint using first 3 pairs
        /*size_t nPairs = std::min(BluePosition.size(), YellowPosition.size());
        if (nPairs >= 3) {
          // Using the first 3 closest pairs gives a more robust path estimate
          cv::Point2f sumMidpoints(0, 0);
          cv::Point2f _;
          for (size_t i = 0; i < 3; ++i)
          {
          _ = BluePosition[i] + YellowPosition[i];
          sumMidpoints += 0.5 * (_);
          }
          midPoint = sumMidpoints * (1.0f / 3.0f);
          validMidpoint = true;
        }
        else if (!BluePosition.empty() && !YellowPosition.empty())
        {
          cv::Point2f _ = BluePosition[0] + YellowPosition[0];
          midPoint = 0.5 * (_);
          validMidpoint = true;
        } 
        else if (!YellowPosition.empty())
        {
          cv::Point2f _ = YellowPosition[0];
          midPoint = _ + cv::Point2f(+200.0f, 0);
          validMidpoint = true;
        } 
        else if (!BluePosition.empty())
        {
          cv::Point2f _ = BluePosition[0];
          midPoint = _ + cv::Point2f(-200.0f, 0);
          validMidpoint = true;
        }*/

        // ====================== TRAJECTORY ends ====================== //

        // ====================== CONTROLLER =========================== // 
        if (validMidpoint) {

          /*cv::Mat R_transpose = R.t();

          cv::Point3d midPointWorld(midPoint.x, midPoint.y, 0.0);

          cv::Point2f midPointCamera = project3DPointBackToImage(midPointWorld);

          std::cout << midPointCamera;*/

          float centerX = img.cols / 2.0f;
          float error = midPoint.x - centerX;

          // -------- PI -------- //
          float Kp = 0.03f, Ki = 0.001f;
          integral += error;
          float steeringDeg = Kp * error + Ki * integral;

          //float steeringStraightDeg = 0.0f; // trial to go straight
          float steeringRad = steeringDeg / 180.0f * static_cast<float>(M_PI); // convert to radians
          steeringRad = std::clamp(steeringRad, -0.663f, 0.663f); // safety clamp for 38 deg max

          // -------------------- //

          // -- Sending commands -- //
          static float base_thrust = 0.1f;
          
          float leftsteer = base_thrust+(0.1f+steeringRad/static_cast<float>(M_PI));
          float rightsteer = base_thrust+(0.1f-steeringRad/static_cast<float>(M_PI));

          std::pair<float,float> drivevalues = {leftsteer,rightsteer};
          cluon::data::TimeStamp sampleTime = cluon::time::now();

          opendlv::proxy::PedalPositionRequest pprl;
          pprl.position(drivevalues.first);
          od4.send(pprl, sampleTime, 0);

          opendlv::proxy::PedalPositionRequest pprr;
          pprr.position(drivevalues.second);
          od4.send(pprr, sampleTime, 1);

          std::cout << "[DEBUG] Sent steering = " << steeringRad << " rad (" << steeringDeg << " deg)" << std::endl; // Print debug commands
          std::cout << "[DEBUG] Sent left pedal = " << leftsteer << " right pedal (" << rightsteer << std::endl; // Print debug commands

          // ---------------------- //


          // --- Printing trajectory -- //
          cv::circle(img, midPoint, 5, cv::Scalar(0, 0, 255), -1); // Print the computed midPointCamera in red
          float try_centerX = img.cols / 2.0f;
          cv::circle(img, cv::Point2f(try_centerX, img.rows - 5), 5, cv::Scalar(255, 0, 255), -1); // Print the robot center of vision in pink
          cv::Point2f robotBase(img.cols / 2.0f, img.rows);
          cv::line(img, robotBase, midPoint, cv::Scalar(255, 255, 0), 2); // Print a line that connect robot center to trajectory midpoind
          // ------------------------- //
        }
        else
        {
          integral = 0.0f;

          std::pair<float,float> drivevalues = {0.1f,0.15f};
          cluon::data::TimeStamp sampleTime = cluon::time::now();

          opendlv::proxy::PedalPositionRequest pprl;
          pprl.position(drivevalues.first);
          od4.send(pprl, sampleTime, 0);

          opendlv::proxy::PedalPositionRequest pprr;
          pprr.position(drivevalues.second);
          od4.send(pprr, sampleTime, 1);
        }
        // ===================== CONTROLLER ends =========================== //

        // ===================== IMAGE printing ============================ //
        // Draw contours in green for boith images
        for (const auto& cnt : contoursBlue)
        {
          if (cv::contourArea(cnt) > 100)
          {
            cv::drawContours(img, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
            cv::drawContours(combinedCones, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
          }
        }

        for (const auto& cnt : contoursYellow)
        {
          if (cv::contourArea(cnt) > 100)
          {
            cv::drawContours(img, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
            cv::drawContours(combinedCones, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0, 255, 0), 2);
          }
        }

        // Draw centroids in green
        /*for (const auto& c : blueCentroids)
        {
          cv::circle(img, c, 4, cv::Scalar(0, 255, 0), -1);
          cv::circle(combinedCones, c, 4, cv::Scalar(0, 255, 0), -1);
        }
        for (const auto& c : yellowCentroids)
        {
          cv::circle(img, c, 4, cv::Scalar(0, 255, 0), -1);
          cv::circle(combinedCones, c, 4, cv::Scalar(0, 255, 0), -1);
        }*/


        cv::imshow("Processed Image", img);
        //cv::imshow("Combined cones", combinedCones);

        // ===================== IMAGE printing ends ======================== //
        
        if (verbose) {
          cv::waitKey(1);
        }

        //{
        // std::lock_guard<std::mutex> lck(distancesMutex);
        // std::cout << "front = " << front << ", rear = " << rear
        //           << ", left = " << left << ", right = " << right << "." << std::endl;
        //}

        //od4.send(opendlv::proxy::AngleReading().angle(123.45f));
      }
    }
    retCode = 0;
  }
  return retCode;
}

# 🤖 Autonomous Robot – OpenDLV

🧠 Designed and implemented a modular real-time behavior control for a ground robot, integrating sensor fusion, computer vision, and decision-making algorithm into a cohesive embedded system.

--- 

## The projects

- The maze solver can autonomously navigate and escape from a maze while collecting all the objects along the way.
- The self-sustained robot is capable of picking up objects and returning to the charging station every 2 minutes.
- The track follower reliably follows a path marked by alternating blue and yellow cones.

---

## 🚀 Features

- 🧠 Reactive obstacle avoidance logic implemented in C++
- 🛰️ Message-based communication using OpenDLV
- 🐳 Docker integration for consistent development and simulation
- 🎥 Support for video-based perception or sensor input (via extended nodes)
- 🧪 Simulation-ready with included task2 test commands

---

## Track follower Video Demo

Under very low-light conditions, careful tuning and repeated use of Data Replay are essential:

<p align="center">
  <a href="https://youtu.be/-OJeIpq-1BA" target="_blank">
    <img src="https://img.youtube.com/vi/-OJeIpq-1BA/hqdefault.jpg" alt="Watch the video" width="800">
  </a>
</p>

---

## 🛠️ Tech Stack

- **C++17**
- **OpenDLV** framework
- **Cluon** messaging library (`cluon-complete-v0.0.145.hpp`)
- **Docker** & `docker-compose`
- **GitLab CI** configuration
- **Simulated task commands** for replay testing

---

## Get into track!

Even under challenging low-light conditions, the robot is able to reliably follow the track!

<p align="center">
  <a href="https://youtube.com/shorts/32uIe9tmV-Q" target="_blank">
    <img src="https://img.youtube.com/vi/32uIe9tmV-Q/hqdefault.jpg" alt="Watch the video" width="800">
  </a>
</p>

--- 

## 📁 Repository Structure

<pre>

├── maze_solver
├── self_sustained_robot
├── track_follower
├── tuning
├── LICENSE
└── README
  
inside a project folder:
  
src/ # Source code and perception node
├── opendlv-perception-helloworld.cpp
├── cluon-complete-v0.0.145.hpp
├── opendlv-message-standard-1.0.odvd
├── terminal_task2_commands.txt
└── terminal_task2_sim_commands.txt

</pre>

Inside the 'tuning' folder, you'll find a very useful tool for image tuning and visualization.

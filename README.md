# 🤖 Autonomous Robot – OpenDLV Obstacle Avoider

A modular autonomous robot system built with the OpenDLV ecosystem and C++. This project enables real-time obstacle avoidance using simulated and real sensor data, designed for Dockerized environments and reproducible robotics experiments.

---

## 🚀 Features

- 🧠 Reactive obstacle avoidance logic implemented in C++
- 🛰️ Message-based communication using OpenDLV
- 🐳 Docker integration for consistent development and simulation
- 🎥 Support for video-based perception or sensor input (via extended nodes)
- 🧪 Simulation-ready with included task2 test commands

---

## Video Demo

Data Replay:

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

<p align="center">
  <a href="https://youtube.com/shorts/32uIe9tmV-Q" target="_blank">
    <img src="https://img.youtube.com/vi/32uIe9tmV-Q/hqdefault.jpg" alt="Watch the video" width="800">
  </a>
</p>

--- 

## 📁 Repository Structure

<pre>

src/ # Source code and perception node
├── opendlv-perception-helloworld.cpp
├── cluon-complete-v0.0.145.hpp
├── opendlv-message-standard-1.0.odvd
├── terminal_task2_commands.txt
└── terminal_task2_sim_commands.txt

recordings/ # Log or sensor data for replay
├── Dockerfile # Container definition
├── docker-compose.yml # Multi-container orchestration
├── CMakeLists.txt # Build system
├── .gitlab-ci.yml # CI configuration
└── LICENSE
</pre>

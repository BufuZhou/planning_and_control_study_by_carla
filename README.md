## Table of Contents

1. [Introduction](#introduction)
2. [Installation](#installation)
3. [Quick Starts](#quick-starts) 
4. [Algorithm Documents](#algorithm-documents)

## Introduction
The main purpose of this project is to learn planning and control in autonomous driving. Currently, this project can only be used for simulation, and the core planning and control algorithm is based on Baidu Apollo.

Because Baidu Apollo is too complex and uses self-developed cyberRT as the middleware. This is not very friendly to beginners. In order to learn planning and control algorithms faster, this project uses the more widely used middleware ros2.Compared with Baidu Apollo's self-developed middleware, ros2 has a lower learning cost and can find a lot of information and demos.

Since this project can only be simulated, the simulator is very important. Here we choose carla. carla is an open source simulator that can simulate autonomous driving scenarios very realistically. It supports flexible specification of sensor suites, environmental conditions, full control of all static and dynamic actors, maps generation and much more.
[lateral vehicle dynamics](https://carla.readthedocs.io/en/latest/img/tuto_G_getting_started/flying_spectator.gif)
For Baidu Apollo's original planning and control algorithms, the original code is simplified, and a lot of algorithm-related documents are supplemented, so that these algorithms can be understood faster and better.

## installation
- [Installation Instructions](docs/01_install/)- **This step is required**

## quick-starts
- [Installation Instructions](docs/02_quick_start/)

## algorithm-documents
- [Control](docs/07_control/)











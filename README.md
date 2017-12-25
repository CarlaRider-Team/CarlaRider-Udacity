## Udacity Self-Driving Car Nanodegree
## Capstone Project

### CarlaRider Team
| Name                       | Email                    |
|:---------------------------|:-------------------------|
| M. Fatih Cırıt (Team Lead) | mfcirit@gmail.com        |
| Kang Shin                  | kshin33@gmail.com        |
| Pratik Thaker              | pratikthaker@gmail.com   |
| Rudy Pena                  | rudypenajr@gmail.com     |
| Andres Munoz               | munoznajar@gmail.com     |

### Requirements
What we have done in this project: 

* Follow given waypoints in the simulator.
* Detect and classify traffic lights.
* Stop at red and go at green.
* Publish brake, steering, throttle commands over ROS at 50 Hz.
* When DBW is toggled, stop/restart controllers.
* Don't exceed the given max speed.

### Traffic Light Detection

We have used [Single Shot Detector Inception v2 model](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) that was pretrained with COCO dataset. And we have further trained it with the help of [TF Object Detection API](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md) with udacity's simulator and real images given in the bag file.
Detailed results are in: [tl_detection.ipynb](https://github.com/CarlaRider-Team/CarlaRider-Udacity/blob/master/tl_detection.ipynb)

### Results 

Car is able to perform all the requirements that were asked. Here is a vid from simulator:

[![Simulator](http://img.youtube.com/vi/RTB42JcAkKk/0.jpg)](http://www.youtube.com/watch?v=RTB42JcAkKk "Simulator")

---

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

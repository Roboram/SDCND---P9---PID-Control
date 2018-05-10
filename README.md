# PID Control

This project consists of a c++ implementation of PID controller to control the steering angle of a car using the Udacity simulator. The main goals of this project is to develop a c++ PID controller that successfully drives the vehicle around the track (Udacity simulator). Figure 1 depicts the car being controlled by the PID controller. 

## 1.Access 

The source code for this project is available at [project code](https://github.com/otomata/CarND-Controls-PID).

## 2.Files

The following files are part of this project: 
* pid.cpp:   PID controller class definition;
* main.cpp:  main file that integrates the PID controller with the simulator.
* images: 
** kp.mp4:  P controller (mannually tuned) driving the car;
** kd.mp4:  PD controller (mannually tuned) driving the car;
** ki.mp4:  PID controller (mannually tuned) driving the car;
** pid.mp4:  Final PID controller (auto tuned) driving the car;

### 2.1 Dependency

This project requires the following packages to work:
* Udacity Simulator [https://github.com/udacity/self-driving-car-sim/releases/](https://github.com/udacity/self-driving-car-sim/releases/);
* cmake 3.5 or above;
* make 4.1 or above;
* gcc/g++: 5.4 or above;
* uWebSocketIO;

### 2.2 WebSocketIO

This project uses the open source package called WebScokectIO to facilitate the communication between the PID controller and the Udacity Simulator. To install all the websocketio libs, execute the script ``install-ubuntu.sh`` from the project repository directory.

## 3.How to use this project

To run this project, you first need to compile the code. After the code is compiled, please, run the Udacity simulator and the PID binary created at the build folder.

### 3.1 Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid
6. Run the Udacity Simulator (./term2_simulator)

## 4.PID 

The PID (Proportional, Integral and Derivative) controller is a closed loop controller widely used by the industry.  It computes the system input variable from the error e(t) between the desired set point and the system output (process variable). The control response (system input) is calculated by applying the proportional, derivative and integral gains over e(t).

For this project, the PID controller is used to control the steering angle from the ``cross track error`` e(t) (car distance from the track center). Therefore, the system input is the steering angle, the output is the car distance from the center and the setpoin it zero (closest to the center as possible). 

![equation](http://latex.codecogs.com/gif.latex?%5Calpha%20%3D%20-K_pe%28t%29%20-K_d%5Cfrac%7Bde%28t%29%7D%7Bdt%7D%20-%20K_i%5Csum%20e%28t%29)

### 4.1 Kp (Proportional Gain)

The Kp gain results into a proportional control response. In the context of this project, it means that the steer input is in proportion to the Cross Track error. However, the proportional control alone results into a marginally stable system because the car will never converge to the set point, it will slightly overshoot. In addition, increasing the value of Kp will make the car react faster but it will also oscillate more around the center lane (set point).

### 4.2 Kd (Derivative Gain)

The Kd gain considers the rate change of error and tries to bring this rate to zero. The derivative gain complements the proportional output by reducing the overshoot, its main goal is to flattening the car trajectory.


### 4.3 Ki (Integral Gain)

The Ki gain reduces the persistent error, i.e. the accumulated error. The integral gain helps the controller to deal with the  ``systematic bias`` problem witch leads to a systematic error. The Ki gain should help to remove the residual error to approximate the car near to the center of the track. It is important to mention that the car oscillates a little bit more than PD controller because the ki gain has been manually tuned.

## 5.PID Tunning


The final tuned PID controller successfully drives the car around the track without pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans where in the vehicle). The file PID parameters are shown below:

* Kp: 0.11
* Ki: 0.00415711
* Kd: 0.00087019


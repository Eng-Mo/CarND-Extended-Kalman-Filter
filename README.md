
# CarND Extended Kalman Filter Project

[//]: # (Image References)
[image1]: ./out_Images/EKF-L.png
[image2]: ./out_Images/EKF-R.png
[image3]: ./out_Images/EKF-LR.png
[image4]: ./out_Images/EKF-LR_Reverse.png



---
This project is an implementation of a sensor fusion algorithm based on Extended Kalman filter for object tracking combining Lidar and Radar sensors.

---
### Project Goals
This project aims to explore Kalman filter contribution in sensor fusion while combining two different sensors, Lidar and here I assume a linear motion model, and Radar sensor and I use non-linear motion model and the steps as following:

1. Initialize the Kalman Parameter and initial state.
2. Predict the next state and covariance.
3. Update the state based on sensor type.

### Build instructions
1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./ExtendedKF

---


## Results

The Algorithm is tested with single sensor and both Lidar and radar sensor. the images below shows the tracking performance in each test as the vertical axis in the y axis and the horizontal axes in the x axis and the Residual error is calculated by root mean square error. the system shows higher RMSE in test 1 and 2 in using either radar or lidar and the accuracy in Lidar is higher than the radar in estimating the position but almost the same in estimating velocity.  In combining both sensors the accuracy increased and RMSE decreased to below the required level. 

RMSe| Laser | Radar| Laser and Radar
------------|------------------------
X   | .1840 |.2508 |  .0974
Y   | .1534 |.2858 |  .0855
vX  | .6056 |.6075 |  .4517
vY  | .0486 |.6942 |  .4404

![alt text][image1]
![alt text][image2]
![alt text][image3]
![alt text][image4]

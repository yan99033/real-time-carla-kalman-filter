# real-time-carla-kalman-filter
A tutorial to understand Kalman filter with real-time trajectory estimation in Carla simulator. 

I struggled to wrap my head around the implementation of Kalman filter. While you can find a lot of resources explaining how Kalman filter works in theory, most of them come with a toy example or a simplified application that does not even apply to challenging real-word situations. I hope that this project can serve as an educational purpose to gain a deeper understanding about Kalman filter and its limitations. 

## TODO

- [ ] Setup system environment (download CARLA and run CARLA)
- [ ] Put a car in an environment 
- [ ] Keyboard control to drive the car in an environment
- [ ] get and visualize LiDAR scan
- [ ] get and visualize image
- [ ] Implement Kalman filter
  - [ ] Prediction
    - [ ] Figure out the vehicle state, motion model, and motion model Jacobians
    - [ ] Estimate the vehicle pose using steering angle and accelerator
  - [ ] Update 
    - [ ] Figure out the measurement and measurement model Jacobians
    - [ ] Estimate the vehicle pose using PnP (matched ORB feature and 3D-2D pose estimation)
   - [ ] Complete prediction-update cycle
- [ ] visualize concatenated camera poses to form a trajectory in Pangolin
- [ ] (possible extension) Use GPS signal to measure global vehicle location (not pose)

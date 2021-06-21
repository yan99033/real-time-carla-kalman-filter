# real-time-carla-kalman-filter
A tutorial to understand Kalman filter with real-time trajectory estimation in Carla simulator. 

I struggled to wrap my head around the implementation of Kalman filter. While you can find a lot of resources explaining how Kalman filter works in theory, most of them come with a toy example or a simplified application that does not even apply to challenging real-word situations. I hope that this project can serve as an educational purpose to gain a deeper understanding about Kalman filter and its limitations. 

## Dependencies
* CARLA (tested 0.9.11)
* Python 3.7
* pygame (>=2.0)


## Download instructions
1. Download [CARLA 0.9.11](https://github.com/carla-simulator/carla/releases/tag/0.9.11)
2. `cd CARLA_0.9.11 && cd PythonAPI`
3. `git clone https://github.com/yan99033/real-time-carla-kalman-filter.git`

## How to use
1. Launch Carla simulator
```sh
cd CARLA_0.9.11
./CarlaUE4.sh
```
2. Run Kalman filter
```sh
cd PythonAPI && cd real-time-carla-kalman-filter
python run_kalman_filter.py
```

## TODO

- [x] Setup system environment (download CARLA and run CARLA)
- [x] Put a car in an environment 
- [x] Keyboard control to drive the car in an environment
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

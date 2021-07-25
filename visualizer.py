# Author: Shing-Yan Loo (lsyan@ualberta.ca)
# Visualize the sensor readings

import matplotlib
matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

import time
import math

from numpy.core.shape_base import block

def add_sensor_reading(sensor_list, new_reading, keep_last_n):
    """Add new sensor reading to the list and only keep the last n
    readings

    :param sensor_list: Container that keeps the sensor reading
    :type sensor_list: list
    :param new_reading: latest sensor reading
    :type new_reading: float
    :param keep_last_n: max number of length for sensor_list
    :type keep_last_n: int
    """
    sensor_list.append(new_reading)

    if len(sensor_list) > keep_last_n:
        # in-place modification
        sensor_list[:] = sensor_list[-keep_last_n:]

def add_to_traj(traj_x, traj_y, traj_z, x, y, z, max_len, min_dist=0.1):
    """Add new xyz position to the trajectory
    Two important considerations:
    1. remove the old points (sliding window) once the 'buffer' is full
    2. don't add the point if it is too close to the previous one

    :param traj: list of x locations
    :type traj: list
    :param traj: list of y locations
    :type traj: list
    :param traj: list of z locations
    :type traj: list
    :param x: x position
    :type x: double
    :param y: x position
    :type y: double
    :param z: z position
    :type z: double
    :param max_len: keep the last n positions
    :type max_len: int
    :param min_dist: min distance to consider adding the new point
    type min_dist: double
    """
    assert len(traj_x) == len(traj_y) and len(traj_y) == len(traj_z)
    # Calculate the distance between the current point and the last point
    # and skip if the distance is too small (e.g., vehicle is stationery)
    if len(traj_x) > 0 and \
        math.sqrt((traj_x[-1] - x)**2 + (traj_y[-1] - y) ** 2 + (traj_z[-1] - z) ** 2) < min_dist:
        return

    # Add new position to the trajectory
    traj_x.append(x)
    traj_y.append(y)
    traj_z.append(z)

    # Keep the lists within the maximum length
    # in-place modification
    if len(traj_x) > max_len:
        traj_x[:] = traj_x[-max_len:]
    if len(traj_y) > max_len:
        traj_y[:] = traj_y[-max_len:]
    if len(traj_z) > max_len:
        traj_z[:] = traj_z[-max_len:]    

def visualizer(visual_msg_queue, quit):
    # Setup layout
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=(0.6, 0.6, 0.6))
    gs = gridspec.GridSpec(4, 5)
    pose_plot = fig.add_subplot(gs[:, :2], projection='3d', facecolor=(1.0, 1.0, 1.0))
    pose_plot.title.set_text('Position')
    imu_plot = fig.add_subplot(gs[:2, 2:4], facecolor=(1.0, 1.0, 1.0))
    imu_plot.title.set_text('IMU')
    gnss_plot = fig.add_subplot(gs[2:, 2:4], projection='3d', facecolor=(1.0, 1.0, 1.0))
    gnss_plot.title.set_text('GNSS')

    # Expand the plot when needed
    pose_plot.autoscale()
    # pose_plot.set_zlim([-2, 10])

    # Trajectory length limit
    max_traj_len = 50000

    # Keep vehicle trajectories
    gt_traj_x = []
    gt_traj_y = []
    gt_traj_z = []

    # GPS trajectory
    gnss_traj_x = []
    gnss_traj_y = []
    gnss_traj_z = []

    # Keep past n IMU sensor reading
    keep_last_n = 50
    t = 0
    ts_imu = []
    acc_x_all = []
    acc_y_all = []
    acc_z_all = []
    gyro_x_all = []
    gyro_y_all = []
    gyro_z_all = []


    # make sure the window is raised, but the script keeps going
    plt.show(block=False)

    while True: # not quit.value:
        if visual_msg_queue.empty():
            time.sleep(0.01)
            continue
        
        msg = visual_msg_queue.get()

        # Visualize vehicle trajectory
        if msg.get('gt_traj') is not None:
            gt_pos = msg['gt_traj']
            add_to_traj(gt_traj_x, gt_traj_y, gt_traj_z, gt_pos[0], gt_pos[1], gt_pos[2], max_traj_len)

            # Clear previous plot
            pose_plot.cla()

            # Update vehicle trajectory
            pose_plot.plot(gt_traj_x, gt_traj_y, gt_traj_z, color='green', linestyle='solid')

        # Visualize IMU reading
        if msg.get('imu') is not None:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = msg['imu']
            add_sensor_reading(acc_x_all, acc_x, keep_last_n)
            add_sensor_reading(acc_y_all, acc_y, keep_last_n)
            add_sensor_reading(acc_z_all, acc_z, keep_last_n)
            add_sensor_reading(gyro_x_all, gyro_x, keep_last_n)
            add_sensor_reading(gyro_y_all, gyro_y, keep_last_n)
            add_sensor_reading(gyro_z_all, gyro_z, keep_last_n)

            ts_imu.append(t)
            if len(ts_imu) > keep_last_n:
                ts_imu = ts_imu[-keep_last_n:]

            # Clear previous plot
            imu_plot.cla()

            # Update IMU reading
            imu_plot.plot(ts_imu, acc_x_all, label='imu_x')
            imu_plot.plot(ts_imu, acc_y_all, label='imu_y')
            imu_plot.plot(ts_imu, acc_z_all, label='imu_z')
            imu_plot.plot(ts_imu, gyro_x_all, label='gyro_x')
            imu_plot.plot(ts_imu, gyro_y_all, label='gyro_y')
            imu_plot.plot(ts_imu, gyro_z_all, label='gyro_z')
            imu_plot.legend(fontsize=14)

        # Visualize GNSS reading
        if msg.get('gnss') is not None:
            gnss = msg['gnss']
            add_to_traj(gnss_traj_x, gnss_traj_y, gnss_traj_z, gnss[0], gnss[1], gnss[2], max_traj_len)

            # Clear previous plot
            gnss_plot.cla()

            # Update GNSS reading
            gnss_plot.plot(gnss_traj_x, gnss_traj_y, gnss_traj_z, color='black', linestyle='solid')
        
        # flush any pending GUI events, re-painting the screen if needed
        fig.canvas.flush_events()
        t += 1

        if not quit.value:
            fig.canvas.draw_idle()
        else:
            plt.close('all')
            print('quiting visualizer loop')
            break
        
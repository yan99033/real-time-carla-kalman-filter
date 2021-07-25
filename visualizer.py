# Author: Shing-Yan Loo (lsyan@ualberta.ca)
# Visualize the sensor readings

import matplotlib
matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

import time

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
            gt_traj_x.append(gt_pos[0])
            gt_traj_y.append(gt_pos[1])
            gt_traj_z.append(gt_pos[2])

            # Clear previous plot
            pose_plot.cla()

            # Update vehicle trajectory
            pose_plot.plot(gt_traj_x, gt_traj_y, gt_traj_z, color='green', linestyle='solid')
            
            # # copy the image to the GUI state, but screen might not be changed yet
            # fig.canvas.blit(fig.bbox)

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
            gnss_traj_x.append(gnss[0])
            gnss_traj_y.append(gnss[1])
            gnss_traj_z.append(gnss[2])

            # Clear previous plot
            gnss_plot.cla()

            # Update GNSS reading
            gnss_plot.plot(gnss_traj_x, gnss_traj_y, gnss_traj_z, color='black', linestyle='solid')

        
        # flush any pending GUI events, re-painting the screen if needed
        fig.canvas.flush_events()
        t += 1

        print('refreshing visualizer....')
        if not quit.value:
            fig.canvas.draw()
        else:
            plt.close('all')
            print('quiting visualizer loop')
            break
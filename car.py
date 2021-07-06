# Author: Shing-Yan Loo (lsyan@ualberta.ca)
# The car class contains the current vehicle state as well as the sensors
# information
# As soon as a sensor reading (be it an image, or IMU reading) is received,
# a callback is received to push the sensor readings to their respective queues
# 
# To get the snapshot of the current sensor reading, call the get_sensor_readings
# method. To make sure that the sensor readings are synchronized, only keep the last
# n timestamps and get the readings at the closest timestamp

import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import numpy as np

import weakref

from queue import Queue
from collections import OrderedDict

class Car:
    def __init__(self, world, client, spawn_point):
        self.client = client

        # Initialize the vehicle and the sensors
        bp_lib = world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('model3')[0]
        camera_bp = bp_lib.filter("sensor.camera.rgb")[0]
        imu_bp = bp_lib.filter("sensor.other.imu")[0]
        gnss_bp = bp_lib.filter("sensor.other.gnss")[0]

        self.vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        # self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        self.vehicle.set_autopilot(True) # False

        self.camera = world.spawn_actor(
            blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=1.6, z=1.6)),
            attach_to=self.vehicle
        )

        self.imu = world.spawn_actor(
            blueprint=imu_bp,
            transform=carla.Transform(carla.Location(x=0, z=0)),
            attach_to=self.vehicle
        )
        self.gnss = world.spawn_actor(
            blueprint=gnss_bp,
            transform=carla.Transform(carla.Location(x=0, z=0)),
            attach_to=self.vehicle
        )

        self.actor_list = [self.vehicle, self.camera, self.imu, self.gnss]

        # Build the K projection matrix:
        # K = [[Fx,  0, image_w/2],
        #      [ 0, Fy, image_h/2],
        #      [ 0,  0,         1]]

        image_w = camera_bp.get_attribute("image_size_x").as_int()
        image_h = camera_bp.get_attribute("image_size_y").as_int()
        fov = camera_bp.get_attribute("fov").as_float()
        focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

        # In this case Fx and Fy are the same since the pixel aspect
        # ratio is 1
        self.K = np.identity(3)
        self.K[0, 0] = self.K[1, 1] = focal
        self.K[0, 2] = image_w / 2.0
        self.K[1, 2] = image_h / 2.0

        # Keep the sensor readings from the callback to queues
        self.image_queue = Queue()
        self.imu_queue = Queue()
        self.gnss_queue = Queue()

        # Hook sensor readings to callback methods
        weak_self = weakref.ref(self)
        self.camera.listen(lambda data : Car.cam_callback(weak_self, data))
        self.imu.listen(lambda data : Car.imu_callback(weak_self, data))
        self.gnss.listen(lambda data : Car.imu_callback(weak_self, data))

    def destroy(self):
        self.camera.destroy()
        self.imu.destroy()
        self.gnss.destroy()

        self.client.apply_batch([carla.command.DestroyActor(x) 
            for x in self.actor_list if x is not None])

        # if self.vehicle:
        #     self.vehicle.destroy()

    def get_location(self):
        return self.vehicle.get_location()    
        
    @staticmethod
    def cam_callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        self.image_queue.put(data)

    @staticmethod
    def imu_callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        self.imu_queue.put(data)

    @staticmethod
    def gnss_callback(weak_self, data):
        self = weak_self()
        if not self:
            return
        self.gnss_queue.put(data)

    def get_sensor_readings(self, frame):
        """Return a dict containing the sensor readings
        at the particular frame

        :param frame: unique frame at the current world frame
        :type frame: int
        """
        sensors = {'image': None,
                   'imu': None,
                   'gnss': None}

        while not self.image_queue.empty():
            image_data = self.image_queue.get()

            if image_data.frame == frame:
                # Get the raw BGRA buffer and convert it to an array of RGB of
                # shape (image_data.height, image_data.width, 3).
                im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
                im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
                im_array = im_array[:, :, :3][:, :, ::-1]
                sensors['image'] = im_array

                print('found an image', im_array.shape)

                break

        while not self.imu_queue.empty():
            imu_data = self.imu_queue.get()

            if imu_data.frame == frame:
                sensors['imu'] = imu_data
                print('found an imu reading', imu_data)
                break

        while not self.gnss_queue.empty():
            gnss_data = self.gnss_queue.get()

            if gnss_data.frame == frame:
                sensors['gnss'] = gnss_data
                print('found an gnss reading', gnss_data)
                break
                
        return sensors

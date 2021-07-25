# Author: Shing-Yan Loo (lsyan@ualberta.ca)
# The car class contains the current vehicle state as well as the sensors
# information
# As soon as a sensor reading (be it an image, or IMU reading) is received,
# a callback is received to push the sensor readings to their respective queues
# 
# To get the snapshot of the current sensor reading, call the get_sensor_readings
# method. To make sure that the sensor readings are synchronized, only keep the last
# n timestamps and get the readings at the closest timestamp
#
# Credit:
# (Get GNSS reference latitude and longitude) 
# https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
# 
# (Convert GNSS signal to absolute XYZ)
# https://github.com/erdos-project/pylot/blob/master/pylot/utils.py

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
import math

import weakref

import xml.etree.ElementTree as ET

from queue import Queue
from collections import OrderedDict

from util import destroy_queue

class Gnss:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Car:
    def __init__(self, world, client, spawn_point):
        self.world = world
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
        self.camera.listen(lambda data : Car.sensor_callback(weak_self, data, self.image_queue))
        self.imu.listen(lambda data : Car.sensor_callback(weak_self, data, self.imu_queue))
        self.gnss.listen(lambda data : Car.sensor_callback(weak_self, data, self.gnss_queue))

        # Reference latitude and longitude (GNSS)
        self.gnss_lat_ref, self.gnss_long_ref = self._get_latlon_ref()

    def destroy(self):
        self.camera.destroy()
        self.imu.destroy()
        self.gnss.destroy()

        self.client.apply_batch([carla.command.DestroyActor(x) 
            for x in self.actor_list if x is not None])
        
        destroy_queue(self.image_queue)
        destroy_queue(self.imu_queue)
        destroy_queue(self.gnss_queue)


    def get_location(self):
        return self.vehicle.get_location()    
        
    @staticmethod
    def sensor_callback(weak_self, data, queue):
        self = weak_self()
        if not self:
            return
        queue.put(data)

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
                im_array = im_array[:, :, :3] # [:, :, ::-1]
                sensors['image'] = im_array

                self.image_queue.task_done()
                break

            self.image_queue.task_done()


        while not self.imu_queue.empty():
            imu_data = self.imu_queue.get()

            if imu_data.frame == frame:
                sensors['imu'] = imu_data

                self.imu_queue.task_done()
                break

            self.imu_queue.task_done()

        while not self.gnss_queue.empty():
            gnss_data = self.gnss_queue.get()

            if gnss_data.frame == frame:
                
                alt = gnss_data.altitude
                lat = gnss_data.latitude
                long = gnss_data.longitude

                gps_xyz = self.gnss_to_xyz(lat, long, alt)

                sensors['gnss'] = gps_xyz

                self.gnss_queue.task_done()
                break

            self.gnss_queue.task_done()
                
        return sensors

    def gnss_to_xyz(self, latitude, longitude, altitude):
        """Creates Location from GPS (latitude, longitude, altitude).
        This is the inverse of the _location_to_gps method found in
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
        
        Modified from:
        https://github.com/erdos-project/pylot/blob/master/pylot/utils.py
        """
        EARTH_RADIUS_EQUA = 6378137.0

        scale = math.cos(self.gnss_lat_ref * math.pi / 180.0)
        basex = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * self.gnss_long_ref
        basey = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + self.gnss_lat_ref) * math.pi / 360.0))

        x = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * longitude - basex
        y = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + latitude) * math.pi / 360.0)) - basey

        # This wasn't in the original method, but seems to be necessary.
        y *= -1

        return Gnss(x, y, altitude)

    def _get_latlon_ref(self):
        """
        Convert from waypoints world coordinates to CARLA GPS coordinates
        :return: tuple with lat and lon coordinates
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
        """
        xodr = self.world.get_map().to_opendrive()
        tree = ET.ElementTree(ET.fromstring(xodr))

        # default reference
        lat_ref = 42.0
        lon_ref = 2.0

        for opendrive in tree.iter("OpenDRIVE"):
            for header in opendrive.iter("header"):
                for georef in header.iter("geoReference"):
                    if georef.text:
                        str_list = georef.text.split(' ')
                        for item in str_list:
                            if '+lat_0' in item:
                                lat_ref = float(item.split('=')[1])
                            if '+lon_0' in item:
                                lon_ref = float(item.split('=')[1])
        return lat_ref, lon_ref
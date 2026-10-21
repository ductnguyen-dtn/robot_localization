#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from numpy.random import normal as sample_normal
from occupancy_field import OccupancyField
from helper_functions import TFHelper, draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    def __repr__(self):
        return "Particle(x={:.2f}, y={:.2f}, theta={:.2f}, w={:.4f})".format(
            self.x, self.y, self.theta, self.w)

class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        # spread of the initial particle cloud around the starting guess
        self.init_sigma_xy = 0.3        # meters
        self.init_sigma_theta = math.pi/6  # radians

        # odometry motion model noise parameters, same naming as the
        # sample_motion_model_odometry algorithm in Probabilistic Robotics
        # (Thrun, Burgard, Fox). alpha1/alpha2 scale how much a rotation and
        # a translation each add noise to the two rotation terms, and
        # alpha3/alpha4 do the same for the translation term.
        self.alpha1 = 0.2
        self.alpha2 = 0.2
        self.alpha3 = 0.2
        self.alpha4 = 0.2

        # laser likelihood model: how forgiving we are of a beam landing
        # some distance away from the nearest known obstacle. A smaller
        # sigma makes the filter more confident, but also more brittle to
        # a slightly wrong map or a noisy reading.
        self.laser_sigma_hit = 0.2      # meters
        # only use every nth beam when scoring a particle against the scan,
        # since checking all 360+ beams against every particle is the main
        # cost of the whole filter. This is the accuracy/computation
        # tradeoff the assignment calls out directly.
        self.laser_beam_stride = 12

        # after resampling, nudge each particle a little so the cloud
        # doesn't collapse onto a handful of exact duplicates
        self.resample_sigma_xy = 0.02    # meters
        self.resample_sigma_theta = math.radians(2)

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        if self.particle_cloud:
            # weighted mean of x and y is straightforward. theta needs a
            # circular mean instead of a plain average, or two particles
            # at, say, theta = -3.13 and theta = 3.13 (which are almost the
            # same heading) would average out to roughly 0, the opposite
            # direction from either of them.
            mean_x = sum(p.w*p.x for p in self.particle_cloud)
            mean_y = sum(p.w*p.y for p in self.particle_cloud)
            sin_sum = sum(p.w*math.sin(p.theta) for p in self.particle_cloud)
            cos_sum = sum(p.w*math.cos(p.theta) for p in self.particle_cloud)
            mean_theta = math.atan2(sin_sum, cos_sum)
            self.robot_pose = Particle(x=mean_x, y=mean_y, theta=mean_theta).as_pose()
        else:
            self.robot_pose = Pose()

        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose,
                                                            self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        dx, dy, dtheta = delta
        trans = math.hypot(dx, dy)

        if trans < 1e-6:
            # the robot barely moved in x,y (it may have only turned in
            # place). atan2(0, 0) is not well defined, so skip straight to
            # treating the whole motion as a rotation instead of a
            # translate-then-rotate, which keeps this from injecting noise
            # based on a meaningless "direction of travel" angle.
            rot1 = 0.0
            rot2 = self.transform_helper.angle_diff(dtheta, 0.0)
        else:
            direction_of_travel = math.atan2(dy, dx)
            rot1 = self.transform_helper.angle_diff(direction_of_travel, old_odom_xy_theta[2])
            rot2 = self.transform_helper.angle_diff(dtheta, rot1)

        # sample_motion_model_odometry (Probabilistic Robotics, table 5.6):
        # treat the odometry reading itself as noisy, and add particle
        # specific noise scaled by how large the motion was. A bigger move
        # gets proportionally more uncertain.
        n = len(self.particle_cloud)
        rot1_noisy = rot1 + sample_normal(0.0, self.alpha1*rot1**2 + self.alpha2*trans**2, n)
        trans_noisy = trans + sample_normal(0.0, self.alpha3*trans**2 + self.alpha4*(rot1**2 + rot2**2), n)
        rot2_noisy = rot2 + sample_normal(0.0, self.alpha1*rot2**2 + self.alpha2*trans**2, n)

        for p, r1, t, r2 in zip(self.particle_cloud, rot1_noisy, trans_noisy, rot2_noisy):
            p.theta = self.transform_helper.angle_normalize(p.theta + r1)
            p.x += t*math.cos(p.theta)
            p.y += t*math.sin(p.theta)
            p.theta = self.transform_helper.angle_normalize(p.theta + r2)

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()

        weights = [p.w for p in self.particle_cloud]
        resampled = draw_random_sample(self.particle_cloud, weights, self.n_particles)

        # draw_random_sample copies whichever particles survive, so a
        # handful of good particles can turn into many exact duplicates.
        # Jitter each copy a little and reset to uniform weight, so the
        # next laser update has a reason to tell duplicates apart again
        # instead of the cloud slowly collapsing to one point.
        n = len(resampled)
        x_noise = np.random.normal(0.0, self.resample_sigma_xy, n)
        y_noise = np.random.normal(0.0, self.resample_sigma_xy, n)
        theta_noise = np.random.normal(0.0, self.resample_sigma_theta, n)
        for p, dx, dy, dtheta in zip(resampled, x_noise, y_noise, theta_noise):
            p.x += dx
            p.y += dy
            p.theta = self.transform_helper.angle_normalize(p.theta + dtheta)
            p.w = 1.0 / n

        self.particle_cloud = resampled

    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading

            This is the likelihood field model from Probabilistic Robotics:
            for each particle, project every laser beam out from its
            hypothesized pose, look up how far that endpoint lands from
            the closest known obstacle in the map, and score the particle
            higher the closer its beams land to a real wall.
        """
        if not self.particle_cloud:
            return

        r = np.asarray(r)
        theta = np.asarray(theta)
        # drop beams with no real return (too close, too far, or inf/nan),
        # and only keep every laser_beam_stride-th beam of what is left.
        # Scoring every particle against every beam is the expensive part
        # of the whole filter, so this is the main knob for the
        # accuracy/computation tradeoff.
        valid = np.isfinite(r) & (r > 0.05)
        r = r[valid][::self.laser_beam_stride]
        theta = theta[valid][::self.laser_beam_stride]
        if len(r) == 0:
            return

        xs = np.array([p.x for p in self.particle_cloud])
        ys = np.array([p.y for p in self.particle_cloud])
        thetas = np.array([p.theta for p in self.particle_cloud])

        # work in log likelihood so that one bad beam does not zero out a
        # particle's weight outright, and so a particle with many
        # consistent beams doesn't underflow to 0 once multiplied together.
        log_weights = np.zeros(len(self.particle_cloud))
        for r_i, theta_i in zip(r, theta):
            hit_x = xs + r_i*np.cos(thetas + theta_i)
            hit_y = ys + r_i*np.sin(thetas + theta_i)
            dist_to_obstacle = self.occupancy_field.get_closest_obstacle_distance(hit_x, hit_y)
            # a beam that lands off the map gets treated as a bad reading,
            # not as "perfectly matches a wall" or "automatically rules the
            # particle out"
            dist_to_obstacle = np.nan_to_num(dist_to_obstacle, nan=3.0*self.laser_sigma_hit)
            log_weights += -(dist_to_obstacle**2) / (2.0*self.laser_sigma_hit**2)

        # shift by the max before exponentiating so the weights stay in a
        # sane floating point range; normalize_particles() (called right
        # after this, in update_robot_pose) takes care of making them sum
        # to 1 again.
        log_weights -= log_weights.max()
        weights = np.exp(log_weights)
        for p, w in zip(self.particle_cloud, weights):
            p.w = w

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        x0, y0, theta0 = xy_theta

        # scatter particles in a Gaussian cloud around the starting guess.
        # This is the "initialize by random sampling" step of the filter.
        xs = np.random.normal(x0, self.init_sigma_xy, self.n_particles)
        ys = np.random.normal(y0, self.init_sigma_xy, self.n_particles)
        thetas = np.random.normal(theta0, self.init_sigma_theta, self.n_particles)

        self.particle_cloud = [Particle(x=xs[i], y=ys[i], theta=thetas[i], w=1.0)
                                for i in range(self.n_particles)]

        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        if not self.particle_cloud:
            return
        total_weight = sum(p.w for p in self.particle_cloud)
        if total_weight <= 0.0:
            # degenerate case, every particle somehow ended up with zero
            # weight. Fall back to a uniform distribution rather than
            # dividing by zero.
            uniform_weight = 1.0 / len(self.particle_cloud)
            for p in self.particle_cloud:
                p.w = uniform_weight
            return
        for p in self.particle_cloud:
            p.w /= total_weight

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

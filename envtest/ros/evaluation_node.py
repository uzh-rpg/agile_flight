import os
import re
import sys
import yaml
import rospy
import numpy as np

from dodgeros_msgs.msg import Command, QuadState
from envsim_msgs.msg import ObstacleArray
from std_msgs.msg import Empty

from uniplot import plot

class Evaluator:
    def __init__(self, config):
        rospy.init_node("evaluator", anonymous=False)
        self.config = config

        self.xmax = int(self.config['target'])

        self.is_active = False
        self.pos = []
        self.dist = []
        self.time_array = (self.xmax+1)*[np.nan]

        self.hit_obstacle = False
        self.crash = 0
        self.timeout = self.config['timeout']
        self.bounding_box = np.reshape(np.array(
            self.config['bounding_box'], dtype=float), (3,2)).T

        self._initSubscribers(config['topics'])
        self._initPublishers(config['topics'])


    def _initSubscribers(self, config):
        self.state_sub = rospy.Subscriber(
                "/%s/%s" % (config['quad_name'], config['state']),
                QuadState,
                self.callbackState,
                queue_size=1,
                tcp_nodelay=True)

        self.obstacle_sub = rospy.Subscriber(
                "/%s/%s" % (config['quad_name'], config['obstacles']),
                ObstacleArray,
                self.callbackObstacles,
                queue_size=1,
                tcp_nodelay=True)

        self.start_sub = rospy.Subscriber(
                "/%s/%s" % (config['quad_name'], config['start']),
                Empty,
                self.callbackStart,
                queue_size=1,
                tcp_nodelay=True)


    def _initPublishers(self, config):
        self.finish_pub = rospy.Publisher(
                "/%s/%s" % (config['quad_name'], config['finish']),
                Empty,
                queue_size=1,
                tcp_nodelay=True)


    def publishFinish(self):
        self.finish_pub.publish()
        self.printSummary()


    def callbackState(self, msg):
        if not self.is_active:
            return

        pos = np.array([msg.header.stamp.to_sec(),
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])
        self.pos.append(pos)

        pos_x = msg.pose.position.x
        bin_x = int(max(min(np.floor(pos_x),self.xmax),0))
        if np.isnan(self.time_array[bin_x]):
            self.time_array[bin_x] = rospy.get_rostime().to_sec()

        if pos_x > self.xmax:
            self.is_active = False
            self.publishFinish()

        if rospy.get_time() - self.time_array[0] > self.timeout:
            self.abortRun()

        outside = ((pos[1:] > self.bounding_box[1,:])
                    | (pos[1:] < self.bounding_box[0,:])).any(axis=-1)
        if (outside == True).any():
            self.abortRun()



    def callbackStart(self, msg):
        if not self.is_active:
            self.is_active = True
        self.time_array[0] = rospy.get_rostime().to_sec()


    def callbackObstacles(self, msg):
        if not self.is_active:
            return

        obs = msg.obstacles[0]
        dist = np.linalg.norm(np.array([obs.position.x,
                                        obs.position.y,
                                        obs.position.z]))
        margin = dist - obs.scale/2
        self.dist.append([msg.header.stamp.to_sec(), margin])
        if margin < 0:
            if not self.hit_obstacle:
                self.crash += 1
                print("Crashed")
            self.hit_obstacle = True
        else:
            self.hit_obstacle = False


    def abortRun(self):
        print("You did not reach the goal!")
        summary = {}
        summary['Success'] = False
        with open("summary.yaml", "w") as f:
            if os.getenv('ROLLOUT_NAME') is not None:
                tmp = {}
                tmp[os.getenv('ROLLOUT_NAME')] = summary
                yaml.safe_dump(tmp, f)
            else:
                yaml.safe_dump(summary, f)
        rospy.signal_shutdown("Completed Evaluation")


    def printSummary(self):
        ttf = self.time_array[-1] - self.time_array[0]
        summary = {}
        summary['Success'] = True if self.crash == 0 else False
        print("You reached the goal in %5.3f seconds" % ttf)
        summary['time_to_finish'] = ttf
        print("Your intermediate times are:")
        print_distance = 10
        summary['segment_times'] = {}
        for i in range(print_distance, self.xmax+1, print_distance):
            print("    %2i: %5.3fs " % (i,self.time_array[i] - self.time_array[0]))
            summary['segment_times']["%i" % i] = self.time_array[i] - self.time_array[0]
        print("You hit %i obstacles" % self.crash)
        summary['number_crashes'] = self.crash
        with open("summary.yaml", "w") as f:
            if os.getenv('ROLLOUT_NAME') is not None:
                tmp = {}
                tmp[os.getenv('ROLLOUT_NAME')] = summary
                yaml.safe_dump(tmp, f)
            else:
                yaml.safe_dump(summary, f)

        if not self.config['plots']:
            return

        print("Here is a plot of your trajectory in the xy plane")
        pos = np.array(self.pos)
        plot(xs=pos[:,1], ys=pos[:,2], color=True)

        print("Here is a plot of your average velocity per 1m x-segment")
        x = np.arange(1,self.xmax+1)
        dt = np.array(self.time_array)
        y = 1/(dt[1:]-dt[0:-1])
        plot(xs=x, ys=y, color=True)

        print("Here is a plot of the distance to the closest obstacles")
        dist = np.array(self.dist)
        plot(xs=dist[:,0]-self.time_array[0], ys=dist[:,1], color=True)

        rospy.signal_shutdown("Completed Evaluation")



if __name__=="__main__":
    with open("./evaluation_config.yaml") as f:
        config = yaml.safe_load(f)

    Evaluator(config)
    rospy.spin()

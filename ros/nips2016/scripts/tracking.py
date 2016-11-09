#!/usr/bin/env python

from nips2016.environment import BallTracking
import rospy


class Tracking(object):
    def __init__(self):
        self.tracking = BallTracking()

    def run(self):
        while not rospy.is_shutdown():
            debug = rospy.get_param('/nips2016/perception/debug', False)
            grabbed, frame = self.tracking.read()

            if not grabbed:
                rospy.logerr("Cannot grab image from webcam, exiting")
                break

            hsv, mask = self.tracking.get_images(frame)
            ball_center = self.tracking.find_center(frame, hsv, mask)

            if debug:
                self.tracking.draw_images(frame, hsv, mask)

if __name__ == '__main__':
    Tracking().run()
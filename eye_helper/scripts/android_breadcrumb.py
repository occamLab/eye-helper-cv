
import rospy
from breadcrumb import Breadcrumb_tracker
from std_msgs.msg import Float64

class Android_breadcrumb(object):

	def __init__(self, tracker):
		self.tracker = tracker
		self.current_yaw = None

        self.last_reading = rospy.Time.now()
        self.target_yaw = None
        self.index = 0 # testing/debugging
        self.rumble_proportion = 0

        # ----------- ROS Publishers/subscribers.
        rospy.init_node("android_breadcrumb")
        rospy.Subscriber("/android_yaw", Float64, self.process_yaw)

    def run(self):
        if self.tracker != None:
            self.tracker.refresh_all()
            self.get_target_from_tracker()
            if self.target_yaw != None:
            	self.check_if_close()


    def get_target_from_tracker(self):
        """
        sets self.target based on values from self.tracker.
        """
        if self.tracker.xy_distance == None or self.tracker.angle_to_go == None:
        	self.target_yaw = None
            return
        # pitch = math.atan2(self.tracker.z_distance, self.tracker.xy_distance)
        yaw = self.tracker.angle_to_go # TODO: incorporate offset.
        self.target_yaw = yaw

    def check_if_close(self):
        """
        if within some [pretty much arbitrary right now] angle of the target, rumbles. else, no rumble.
        """
        if abs(self.current_yaw - self.target_yaw) < 10: # arbitrary
        	return True
        else:
        	return False
        try:
        	use_sockets_to_tell_phone_to_rumble() # TODO: implement
        except:
        	pass

    def process_yaw(self, msg):
    	self.current_yaw = msg.data

if __name__ == "__main__":
	bt = Breadcrumb_tracker()
	ab = Android_breadcrumb(bt)
    while not rospy.is_shutdown():
		ab.run()
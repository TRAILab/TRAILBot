
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rclpy.clock import Clock

head = Header
head._frame_id = 'map'
test = PoseStamped(header=head)
a=1
import rospy
from sensor_msgs.msg import PointCloud2

rospy.init_node("attaching_frame")
pub = rospy.Publisher("/points_raw", PointCloud2, queue_size=1000000)


def callback(data):
    a = PointCloud2()
    a = data
    print("a.header.frame_id: ", a.header)
    a.header.stamp = rospy.Time.now()
    a.header.frame_id = "world"
    print("a.header.frame_id: ", a.header.frame_id)
    pub.publish(a)


rospy.Subscriber("/firefly/vi_sensor/camera_depth/depth/points", PointCloud2, callback)

rospy.spin()
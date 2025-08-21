import rospy
import time
from geometry_msgs.msg import PoseStamped

center_index=0

last_loong_time=time.time()

center_point=[
              (-3.5,-2),
              (-3.5,-2),
              (0.9,0.3),
              (0.9,0.3),
              
 
              (-2.4,4.0),
              (-2.3,4.0)
              ]

def nav_loong():
    global center_index, last_loong_time

    if time.time() - last_loong_time > 6.5:
        center_index = (center_index + 1) % 3



        pub_point(center_point[center_index])


        print(center_point[center_index])


        last_loong_time = time.time()


def hahah():
    global center_index, last_loong_time
    while True:

        if time.time() - last_loong_time > 3.2:
            _x,_y=center_point[center_index]
            print(center_point[center_index])
            print(_x,_y)
            center_index = (center_index + 1) % 6
            last_loong_time = time.time()
            pub_point(_x,_y)
        # pub_point()


def pub_point(_x,_y):
# def pub_point():

    rospy.init_node('sent_point_node', anonymous=True)

    pub=rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.sleep(0.5)

    point=PoseStamped()
    point.header.frame_id='map'
    point.header.stamp=rospy.Time.now()

    point.pose.position.x=_x
    point.pose.position.y=_y
    # point.pose.position.x=1.5
    # point.pose.position.y=1.5
    point.pose.position.z=0.0

    point.pose.orientation.x=0.0
    point.pose.orientation.y=0.0
    point.pose.orientation.z=0.0
    point.pose.orientation.w=1.0



    rate=rospy.Rate(10)

    # while True:

    pub.publish(point)

    rospy.loginfo("Sending goal point")
        # rate.sleep()


if __name__=='__main__':
#    try:
    # pub_point(0,0)
    # nav_loong()
    hahah()
#    except rospy.ROSInterruptException:
#        pass
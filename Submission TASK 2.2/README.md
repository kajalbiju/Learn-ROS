* Creating a catkin workspace  (1.1)
```
    mkdir -p catkin_ws/src  
    cd catkin_ws  
    catkin_make  

    source devel/setup.bash  
```  

* Creating a ROS package  (1.2)
```
    cd catkin_ws/src  
    catkin_create_pkg beginner_tutorials std_msgs rospy roscpp  

    cd catkin_ws
    catkin_make

    source devel/setup.bash
```  

* Running turtlesim  (1.3)
```
    roscore  
    rosrun turtlesim turtlesim_node  
    rosrun turtlesim turtle_teleop_key
```
**Turtle-bot with teleop key**  

<img src="DATA/turtleism.png">

**Talker-Listener example**  (1.4)
* Code
  
```
cd catkin_ws/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
nano beginner_tutorials/src/talker.py

```

* Code for talker.py
```
    #!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        message = "Hello, ROS!"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
chmod +x beginner_tutorials/src/talker.py

```
* Code
  
```
nano beginner_tutorials/src/listener.py


```

* Code for listener.py
```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard: %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

chmod +x beginner_tutorials/src/listener.py

```

<img src="DATA/talker-listener.png">

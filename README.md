## This ros2 package will allow you to save Lidar PointCloud messages as pcd files.

To run, first build the package then source the local environment by doing the following:

First change the topic name in ~ros2_savelidar/ros2_savelidar/pc_saver.py
'''
        self.subscription = self.create_subscription(
            PointCloud2,
            'YOUR_TOPIC_HERE',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
'''

In the ros2_savelidar/ directory, type:

'''
colcon build
cd install
source setup.bash
'''

Then run the package with:
'''
ros2 run ros2_savelidar save_lidar
'''

# If bridging from Gazebo:
If you are getting lidar information from gazebo, you can call the following command to bridge to ros2
'''
ros2 run ros_gz_bridge parameter_bridge <GAZEBO_TOPIC_NAME>@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked
'''

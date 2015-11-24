# imu_visualization
This package can be used to visualize the output of IMUs which are publishing thier data via sensor_msgs/Imu messages.

#use
* add to your launch file the following sniplet making sure to set the variables inside:
```
<node pkg="imu_visualization" type="display_3D_visualization.py" name="<NODE_NAME>" output="screen">
	<param name="imu_topic" value="<TOPIC_TO_WHICH_IMU_IS_PUBLISHING>" />
	<param name="imu_name" value="<NAME_OF_WINDOWs_(usually_the_imus_name)>" />
</node>
```


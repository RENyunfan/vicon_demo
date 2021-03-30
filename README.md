# vicon_demo 使用教程

本功能包中包含四个`rospackage`，分别如下

```bash
├── joy2twist-master
├── polar_controller
├── vicon_bridge-master
└── vicon_repub
```

# 1 joy2twist

#### 功能说明

`joy2twist`是一个关于遥控器公能的话题转换包。功能为将遥控器发出的`sensor_msgs::Joy`类型数据，转换成无人小车需要的控制数据`geometry_msgs/Twist`。

#### 安装依赖

使用前需要安装依赖

```bash
sudo apt-get install ros-melodic-joystick-*
```

注意，该功能包需要与支持`joystick`的遥控器硬件配合使用。

#### 使用方法

打开`./launch/xbox.launch`文件，修改参数如下

```xml
<launch>
    <!-- 根据是否使用时间戳，填写下行value = true或false-->
	<arg name="use_stamped" value="false"> </arg>
    
	<node pkg="joy" name="joy_node" type="joy_node" output="screen" />
    
	<node pkg="joy2twist" name="xbox" type="xbox" output="screen" >
	    <param name="/use_stamped"   value="$(arg use_stamped)"/>
		<!-- 根据需要发布的话题名，填写下行中to后面的内容，例如dashgo小车需要使用的名字为："/smoother_cmd_vel" -->
        <remap from="cmd_vel" to="/smoother_cmd_vel" />
	</node>

</launch>
```

在更改完成后，运行下方命令即可使用

```
roslaunch joy2twist xbox.launch
```



# 2 vicon_bridge

#### 功能说明

[`vicon_bridge`](https://github.com/ethz-asl/vicon_bridge)是由ETHZ ASL实验室开发的工具包，其功能为，将局域网中共享的`vicon`数据转换为ROS中的话题，发布的消息类型为`geometry_msgs/TransformStamped`。

#### 安装依赖

无需特殊的依赖

#### 使用方法

打开`./launch/vicon.launch`文件，修改参数如下

```xml
<launch>
	<node pkg="vicon_bridge" type="vicon_bridge" name="vicon" output="screen">
		<param name="stream_mode" value="ClientPull" type="str" />
		<!--修改下方value后的数值为vicon系统所在的局域网IP地址和端口号（可以从vicon软件所在的电脑上读取），K321实验室的wifi名为zdh，对应ip和端口号如下-->
		<param name="datastream_hostport" value="192.168.0.13:801" type="str" />
		<param name="tf_ref_frame_id" value="/world" type="str" />
		</node>
</launch>
```

在更改完成后，运行下方命令即可在ROS话题广场中获得vicon数据。



# 3 vicon_repub

#### 功能说明

在1.2中，我们介绍了如何使用`vicon_bridge`将动捕数据发布到ROS话题广场。然而，发布话题类型`geometry_msgs/TransformStamped`无法在`Rviz`中进行可视化，同时和我们常用的`nav_msgs/Odometry`信息结构上也略有差异，因此我们通过`vicon_repub`功能包，实现换题类型的转换。

除此之外，`vicon_repub`还提供了一个地图边界绘制功能，通过设置`config/map.yaml`中的地图大小，可以在Rviz中可视化一个方框。如无需方框可以将地图大小设置为0。

#### 安装依赖

无需特殊依赖

#### 使用方法

打开`./launch/vicon.launch`文件，修改参数如下

```xml
<launch>

	<rosparam   command="load" file="$(find vicon_repub)/config/map.yaml"/>
    <rosparam   command="load" file="$(find polar_controller)/config/default.yaml"/>
	
    <node pkg="vicon_repub" name="vicon_repub" type="vicon_repub" output="screen" >
        <!-- 修改下方 to 后面的节点名字为vicon节点的名字 -->
        <remap from="/vicon_input" to="/vicon/vehicle/vehicle" />
         <!-- 修改下方 to 后面的节点名字为你想要发布的节点名字 -->
        <remap from="/vicon_output" to="/vicon_odom" />
    </node>

	<node pkg="rviz" name="rviz" type="rviz" args="-d $(find vicon_repub)/config/default.rviz" />

    <include file="$(find joy2twist)/launch/xbox.launch"/>
    <include file="$(find vicon_bridge)/launch/vicon.launch"/>
    <include file="$(find dashgo_driver)/launch/driver.launch"/>
</launch>

```

随后直接使用启动

```bash
roslaunch vicon_repub vicon.launch
```

`vicon_repub`功能包核心代码如下

```cpp
void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr & msg){
	nav_msgs::Odometry odom_;
	odom_.pose.pose.position.x = msg-> transform. translation.x;
	odom_.pose.pose.position.y = msg-> transform. translation.y;
	odom_.pose.pose.position.z = msg-> transform. translation.z;
	
	odom_.pose.pose.orientation = msg-> transform.rotation;
	odom_.header.stamp = ros::Time::now();
	odom_.header.frame_id = "world";
	odom_pub.publish(odom_);
}
```

即在动捕话题的回调函数中，重新整理消息类型，并发布到ROS话题广场中。



# 4 polar_controller

#### 功能说明

`polar_controller`是一个基于极坐标控制器的定点控制功能包。其中提供了一个`PolarController`类，可以提供极坐标定点控制器的计算。

#### 安装依赖

无需特殊依赖

#### 使用方法

打开`./launch/vicon.launch`文件，修改参数如下

```xml
<launch>
	<rosparam   command="load" file="$(find vicon_repub)/config/map.yaml"/>
	<node pkg="polar_controler" type="controller" name="controller">
		<!-- 修改下方 to 为里程计输入的节点名字 -->
        <remap from="/odom" to="/vicon_odom" />
          <!-- 修改下方 to 为Rviz 2D Nav Goal 输入的名字 -->
		<remap from="/goal" to="/move_base_simple/goal" />
	</node>
    
	<include file="$(find vicon_repub)/launch/vicon.launch"/>
	<include file="$(find joy2twist)/launch/xbox.launch"/>
	<include file="$(find vicon_bridge)/launch/vicon.launch"/>
	<include file="$(find dashgo_driver)/launch/driver.launch"/>
</launch>
```

随后直接使用启动

```bash
roslaunch polar_controller polar_controller.launch
```






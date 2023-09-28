# 2.3 roslaunch로 여러 노드 실행하기

## Section 1. Create the Launch File

이전에 작성한 `publisher_node`와 `subscriber_node`를 실행하기 위해서는 `roscore`를 포함해 3개의 터미널을 사용해야 하는 번거로움이 있었습니다.

여러 개의 노드들을 한 번에 실행해주는 `roslaunch`를 통해 한 개의 터미널로 이전에 작성한 노드들을 실행해보도록 하겠습니다.

**1.1 Create the launch file**

```
roscd im_newbie
mkdir launch
cd launch
touch hell_ros.launch
```

런치파일에 아래와 같이 작성해줍니다.
```xml
<launch>
  <node pkg="im_newbie" type="publisher_node" name="this_is_publisher" output="screen"/>
  <node pkg="im_newbie" type="subscriber_node" name="this_is_subcriber" output="screen"/>
</launch>
```

`package.xml`도 수정합니다.
```xml
	<build_depend>roslaunch</build_depend>
```

`CMakeLists.txt`에 추가해줍니다.
```cmake
find_package(catkin REQUIRED COMPONENTS
  # ...
  roslaunch
)
```

수정이 완료되면 빌드합니다.

**1.2 Run multiple nodes with roslaunch**

이제 작성한 런치파일을 실행해줍니다.
```bash
roslaunch im_newbie hello_ros.launch
```

## Section 2. How to use roslaunch

`roslaunch`는 여러 노드들을 실행시키는 것 외에도 여러 기능들을 가지고 있습니다.

- [Roslaunch tips for larger projects](http://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects)

- [roslaunch](http://wiki.ros.org/roslaunch)

- [roslaunch XML](http://wiki.ros.org/roslaunch/XML)

- [roslaunch의 사용법 및 XML](https://velog.io/@717lumos/roslaunch%EC%9D%98-%EC%82%AC%EC%9A%A9%EB%B2%95-%EB%B0%8F-XML)

위 링크들에 자세하게 설명되어 있으며, 이 중 대표적인 사용방법들을 예시를 통해 알아보도록 하겠습니다.

```xml
<launch>
	<arg name="fcu_url" default="/dev/ttyPixhawk:57600"/>
	<arg name="gcs_url" default="" />
	<arg name="tgt_system" default="1" />
	<arg name="tgt_component" default="1" />
	<arg name="pointcloud_topics" default="[/camera/depth/points]"/>
	<arg name="json_file_path"      default="$(find koreauav2023)/params/stereo_calib.json"/>

	<!-- Launch MavROS -->
	<include file="$(find mavros)/launch/node.launch">
		<arg name="pluginlists_yaml" value="$(find mavros)/launch/px4_pluginlists.yaml" />
		<arg name="config_yaml" value="$(find avoidance)/resource/px4_config.yaml" />
		<arg name="fcu_url" value="$(arg fcu_url)" />
		<arg name="gcs_url" value="$(arg gcs_url)" />
		<arg name="tgt_system" value="$(arg tgt_system)" />
		<arg name="tgt_component" value="$(arg tgt_component)" />
	</include>

	<!-- Launch cameras -->
	<node pkg="tf" type="static_transform_publisher" name="tf_camera" required="true"
		args="0.13 0 0.115 0 0 0 fcu camera_link 10"/>
		
	<include file="$(find koreauav2023)/launch/rs_depthcloud.launch">
		<arg name="required"              value="true"/>
		<arg name="tf_prefix"             value="camera"/>
		<arg name="enable_color"          value="true" />
		<arg name="enable_infra1"         value="false" />
		<arg name="depth_fps"             value="15"/>
		<arg name="json_file_path"           value="$(arg json_file_path)"/>
		<arg name="align_depth"           value="true"/>
	</include>

	<!-- Launch local planner -->
	<node name="local_planner_node" pkg="local_planner" type="local_planner_node" output="screen" required="true" >
		<param name="goal_x_param" value="0" />
		<param name="goal_y_param" value="0"/>
		<param name="goal_z_param" value="3" />
		<param name="accept_goal_input_topic" value="true" />
		<rosparam param="pointcloud_topics" subst_value="True">$(arg pointcloud_topics)</rosparam>
	</node>

	<!-- GPS to ENU-->
	<include file="$(find koreauav_utils)/launch/yaml_to_gps.launch"/>
	<include file="$(find koreauav_utils)/launch/gps_to_enu.launch"/>
</launch>
```

위 런치파일은 mavros를 이용해 플래너와 실제 Pixhawk FC와 연결하는 노드들, 그리고 다른 런치파일들을 포함하고 있습니다.

**2.1 `<node>`**

런치파일은 XML 형식으로 작성되며 `<node>`는 **Section 1**에서 다룬 것 처럼 노드를 실행시킵니다.

이 `<node>` 태그에는 *pkg*, *type*, *name은* 필수로 필요합니다. *pkg*는 실행시킬 노드가 들어가 있는 패키지, *type*은 실행시킬 노드의 이름(rosrun할 때 들어가는 실행명령 이름)이고 *name*은 설정해줄 노드의 이름입니다.

그래서 같은 노드를 여러 번 실행시키는 경우 *name*을 다르게 해주면 됩니다.

이 외에도 `args`, `respawn`, `required`, `ns`, `output` 등이 많이 사용되고 `if`문이나 `laucnh-prefix`도 가능합니다.

`remap`, `rosparam`, `param`등을 통해 노드에서 사용하는 파라미터를 직접 변경해줄 수도 있습니다.

[참고링크](http://wiki.ros.org/roslaunch/XML/node)

**2.2 `<arg>`**

`<arg>`는 추후 다룰 `<param>`과 달리 런치파일에서 사용하는 변수입니다.  `<include>`로 포함시킨 다른 런치파일에 이를 전달할 수도 있고, 다른 노드에 파라미터로 넘겨줄 수도 있습니다.

ROS의 **Parameter Server**에서 사용되는 rosparam이나 노드 내에서 사용되는 param 달리 `<arg>`는 roslaunch로 실행할 때 외부에서 변경해줄 수 있습니다.

예를 들어

```bash
roslaunch koreauav2023 rs_depthcloud.launch --enable_color:=false
```
로 해주면 위에 런치파일에서는 `enable_color`를 `true`로 해두었지만 `false`로 해서 런치파일을 실행시킬 수 있습니다.

[참고링크](http://wiki.ros.org/roslaunch/XML/arg)

**2.3 `<include>`**

`<include>`는 다른 런치 XML 파일을 현재 파일에 포함시키는 태그입니다. 해당 런치파일이 그대로 현재 위치에 복사되어 들어온다고 이해해도 됩니다.

위의 예제파일에서 `<include>`를 사용한 부분은 아래와 같습니다.

- `<include file="$(find mavros)/launch/node.launch">`
- `<include file="$(find koreauav2023)/launch/rs_depthcloud.launch">`
- `<include file="$(find koreauav_utils)/launch/yaml_to_gps.launch"/>`

`$(find pkg-name)`으로 빌드되어 있는 ROS 패키지 경로를 연결할 수 있고, 여기 하위에 있는 런치파일들을 불러올 수 있습니다.

마찬가지로 `<arg>`태그로 선언되어 있는 변수명들을 상위 런치파일에서 수정해줄 수 있습니다.

[참고링크](http://wiki.ros.org/roslaunch/XML/include)

**2.4 `<param>` and `<rosparam>`**

두 태그 모두 rosparam에 사용되지만 약간의 차이는 있습니다. 실제 사용되는 예시는 추후 코드를 통해 ROS 파라미터를 사용할 때 다뤄보도록 하겠습니다.

자세한 내용은 아래 링크를 참고해주세요.

- [`<param>`](http://wiki.ros.org/roslaunch/XML/param)

- [`<rosparam>`](http://wiki.ros.org/roslaunch/XML/rosparam)

---

아래 링크에 한국어 자료로 위에 설명한 내용을 더 구체적이고 상세하게 설명되어 있습니다.

[roslaunch의 사용법 및 XML](https://velog.io/@717lumos/roslaunch%EC%9D%98-%EC%82%AC%EC%9A%A9%EB%B2%95-%EB%B0%8F-XML)
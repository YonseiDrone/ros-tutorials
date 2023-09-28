# A ROS Noetic Tutorial for Beginners

![Title](./assets/banner_test_yonseidrone.png)

**연세드론 부원을 위한 ROS1 Noetic 튜토리얼입니다.**

!!! abstract "Table of Contents"

	1. [**Introduction**](./01_Introduction/1_introduction.md)

		1.1 ROS Noetic 설치하기
		
		1.2 워크스페이스 설정 및 환경 구성

		1.3 shell파일로 ROS Noetic 설치하기

	2. **ROS Noetic 기초**
			
		2.1 [간단한 Publisher, Subscriber 노드 작성하기](./02_ros_noetic_basic/1_simple_pub_sub_node.md)

		2.2 [메세지에 대한 이해와 커스텀 메세지 다루기](./02_ros_noetic_basic/2_custom_msg.md)

		2.3 [roslaunch로 여러 노드 실행하기](./02_ros_noetic_basic/3_roslaunch.md)

		2.4 [ros Time, Sleeping, Rates and Spinning](./02_ros_noetic_basic/4_rostime_sleep_and_spin.md)

	3. **Turtlesim**

		3.1 [turtlesim 실행하고 키보드로 조작하기](./03_turtlesim/1_turtlesim_teleop.md)

		3.2 [직접 작성한 코드로 제어하기 (예제 #1)](./03_turtlesim/2_turtlesim_writeteleop.md)

		3.3 [서비스에 대한 이해와 상대 위치로 제어하기 (예제 #2)](./03_turtlesim/3_turtlesim_service_launch.md)

	4. **Gazebo**

		4.1 Gazebo 기초

		4.2 원하는 모델과 월드 제작하기

		4.3 Simulation

	5. **MAVROS**

		5.1 MAVROS로 작업하기

		5.2 드론 시뮬레이션 설정하기

		5.3 ROS Noetic으로 드론 제어하기 

!!! example "예제 List"

	[#1 키보드로 turtlesim 거북이 제어하기](https://yonseidrone.github.io/ros-tutorials/03_turtlesim/2_turtlesim_writeteleop/#section-2-mission-1)
	[#2 turtlesim 2번째 거북이 소환하여 상대 위치로 제어하기, 1번 예제 노드와 런치파일 만들기](https://yonseidrone.github.io/ros-tutorials/03_turtlesim/3_turtlesim_service_launch/#section-2-mission-2)
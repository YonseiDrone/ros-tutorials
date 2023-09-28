# 3.3 서비스에 대한 이해와 turtlesim 상대 위치로 제어하기(예제 #2)

앞서 키보드로 제어하는 노드에 더해 새로운 노드를 작성하겠습니다.

서비스에 대해 배우고, 거북이를 한 마리 더 소환하여 turtle2는 turtle1의 위치를 가지고 제어해보겠습니다.

그리고 여러 노드들을 런치파일로 만들어 한번에 실행해보겠습니다.

## Section 1. Services

**1.1 Introduction**

**토픽**은 publish / subscribe로 여러 개의 노드끼리 통신할 수 있지만 한 방향으로만 메세지를 전달 혹은 수신합니다.

request / reply 방식으로 활용되는 것이 바로 **서비스**입니다. **토픽**이 *msg*라는 타입을 사용하는 것처럼, **서비스**는 *srv*라는 타입을 사용합니다.

*srv*는 request와 response 메세지를 가지게 됩니다.

**1.2 Client**

서비스 호출하는 방법은 2가지가 있으며 주로 후자를 사용합니다.

방법 1. Bare
```cpp
ysdrone_msgs::DroneCommand srv;
ros::service::call("/drone_command", srv);
```

방법 2. Handle
```cpp
ros::ServiceClient client = nh.serviceClient<ysdrone_msgs::DroneCommand>("/drone_command");
ysdrone_msgs::DroneCommand srv;
client.call(srv);
```
2번째 방법이 선언된 NodeHandle을 사용하며 주로 클래스를 사용하기 때문에 이 방식으로 사용합니다.

Client는 서비스 요청 메세지를 보내고 응답을 기다리는 주체입니다.


**1.3 Server**

서비스가 호출되면 callback함수를 실행하며 응답하는 Server는 아래와 같이 사용합니다.
```cpp
bool commandCallback(ysdrone_msgs::DroneCommand::Request& request, ysdrone_msgs::DroneCommand::Response& response)
{
    mission = req.command;
    ROS_INFO("[Building Search] Mission set to %d", mission);
    return true;	
}
ros::ServiceServer nh.advertiseService("/drone_command", commandCallback);
```
Client에서 호출한 `/drone_command`가 수신되면 콜백함수가 호출됩니다.

subscriber와 같이 콜백함수는 예시와 같이 정해진 인자를 받고 boolean 타입으로 반환합니다.

클래스에서는 callback함수를 약간 다르게 써야하지만 이는 C++관련 지식이므로 추후에 다룰 예정입니다.

공식 문서에서 다룬 자세한 내용은 [Services](http://wiki.ros.org/Services)를 참고합니다.

**1.3 CLI**

이제 Service와 관련된 명령어들을 살펴보도록 하겠습니다.

현재 사용가능한 서비스 목록을 살펴볼 때는 아래 명령어를 사용합니다
```bash
rosservice list
```

`find`, `info`, `type` 등의 명령어들도 있지만 자세한 내용은 [링크](http://wiki.ros.org/rosservice)로 대체하고 `rosservice call`에 대해서만 살펴보겠습니다.

`rosservice call <service_name> <service-args>`로 사용하는 이 명령어는 위 **1.2 Usage**에서 다룬 Client와 같은 기능입니다.

```bash
# 예시
rosservice call /drone_command 0
```

이러한 명령어는 아래와 같이 런치파일에서도 사용 가능합니다.

```xml
<?xml version="1.0" encoding="utf-8"?>
<launch>
	<!-- 생략... -->
	<node pkg="rosservice" type="rosservice" name="rosservice" args="call /spawn 2.5 2.5 0 turtle2" />
</launch>
```

## Section 2. Mission #2

앞서 [Mission #1](https://yonseidrone.github.io/ros-tutorials/03_turtlesim/2_turtlesim_writeteleop/#section-2-mission-1)에서 만든 키보드 제어 노드와 함께 배운 내용을 복습해보겠습니다.

이 예제에서는 두 번째 거북이를 소환하고 이 거북이를 첫 번째 거북이의 위치를 이용해 제어할 것입니다.

첫 번째 거북이는 앞서 만든 키보드 제어를 통해 이동시킵니다.

!!! info "예제 설명 및 조건"
	- 클래스 작성 없이 .cpp 파일 하나만을 이용해 노드를 작성합니다.
	- `main()`함수 안에 `ros::init`과 `ros::NodeHande`을 선언합니다.
	- 두 번째 거북이를 Service를 이용해 선언합니다. 아래 코드를 참고하세요.
	- 첫 번째와 두 번째 거북이의 위치를 받는 각각의 Subscriber와 두 번째 거북이에게 속도 명령을 주는 Publisher를 선언합니다. [turtlesim 패키지 참고](http://wiki.ros.org/turtlesim)
	- 각각의 콜백함수에서 두 거북이의 위치를 전역변수로 받습니다.
	- while문 안에서 두 번째 거북이를 제어하는 토픽에 맞는 메세지 타입 `geometry_msgs/Twist`의 값을 두 거북이의 상대거리로 넣어줍니다. (혹은 다른 아이디어가 있으면 사용해도 좋습니다.)
	- 토픽을 publish합니다.
	- `ros::Rate`로 주기를 설정하고 while문 마지막에는 `ros::spinOnce()`와 `sleep()`을 넣어줍니다.

!!! example "두 번째 거북이 소환하기"
	```cpp
	#include <turtlesim/Spawn.h>

	ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn spawn;
	spawn.request.x = 2;
	spawn.request.y = 2;
	spawn.request.theta = 0;
	if (spawnClient.call(spawn))
	{
		ROS_INFO("Successfully spawned turtle2");
	}
	else
	{
		ROS_ERROR("Failed to spawn turtle2");
		return 1;
	}
	```

소스코드 작성을 완료하면 `CMakeLists.txt`를 수정하고 빌드해줍니다.

`roscore`와 각각의 노드를 실행하여 코드를 적절하게 작성하였는데 테스트 해봅니다.

!!! success "목표"
	**1. 첫 번째 거북이 `turtle1`은 앞서 만든 키보드 노드로 제어합니다.**

	**2. 두 번째 거북이 `turtle2`는 자신의 위치와 첫 번째 거북이의 위치의 차이 이용해 첫 번째 거북이를 따라갑니다.**

	**3. 여러 개의 노드를 사용하므로 테스트 완료 후에는 `turtlesim`과 모든 노드를 같이 실행할 수 있는 런치파일을 작성합니다.**
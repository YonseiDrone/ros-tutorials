# 2.4 ros Time, Sleeping, Rates and Spinning

## Section 1. ROS Time and Duration

**1.1 Time**

ROS에서는 자체적으로 내장된 `ros::Time`과 `ros::Duration`을 제공합니다. `Time`은 특정 시점의 시각이고, `Duration`은 일정 기간의 시간을 의미합니다.

자료형은 아래와 같이 구성됩니다.
```
int32 sec
int32 nsec
```

따라서 $\text{time}=\text{sec}+\text{nsec}*10e^{-9}$으로 전체 시간을 얻을 수 있습니다.

현재 ROS 시간을 얻을 때는

```cpp
ros::Time current_time = ros::Time::now();
```
을 사용합니다.

주로 이런 표현은 메세지 자료형 중 [`std_msgs/Header`](http://docs.ros.org/en/api/std_msgs/html/msg/Header.html)의 `stamp`에 값을 넣을 때 사용합니다.

ROS 환경을 시뮬레이션에서 사용할 때는 메세지가 publish된 시간이 시뮬레이션 내의 시간과 맞아야 하므로 `Header`를 포함한 메세지들을 사용합니다.

아래는 위치와 관련된 토픽에 사용되는 메세지 타입인 [`geometry_msgs/PoseStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)를 publish하는 코드 예시입니다.

```cpp
geometry_msgs::PoseStamped msg;
msg.header.stamp = ros::Time::now();
msg.header.frame_id = "fmu";

# geometry_msgs/Point
msg.pose.position.x = 2;
msg.pose.position.y = 0;
msg.pose.position.z = 0;
# geometry_msgs/Quaternion
msg.pose.orientation.x = 0;
msg.pose.orientation.y = 0;
msg.pose.orientation.z = 0;
msg.pose.orientation.w = 1;

pose_pub.publsh(msg);
```
주의할 점은 ROS에서 사용하는 time 자료형은 실수형과 다릅니다. 따라서 실수형으로 바꾸고 싶은 경우에는 `.toSec()`을 사용합니다.

**1.2 Duration**

`ros::Duration`은 아래와 같이 사용하며, 보통 시간에 더해주거나 `sleep`에 사용합니다.

들어가는 실수형은 초 단위를 사용합니다. 추후 다룰 `Rate`와 종종 혼동하니 잘 기억해두는 것이 좋습니다.

```cpp
ros::Duration d(1.0); // 1초
ros::Time later = current_time + d;
```

## Section 2. Rate and sleep

토픽을 publish하는 주기를 일정하게 맞추고자 노드를 일정 시간 동안 중지하는 경우가 있습니다. (대부분의 센서 데이터 수신 혹은 제어 명령 주기는 10Hz~ 이상을 사용합니다.)

이러한 경우에는 `ros::Rate`를 사용합니다.

가장 많이 사용하는 예시입니다.

```cpp
ros::Rate rate(10); // 10 Hz
while (ros::ok()) {
	// ... do something
	rate.sleep();
}
```
이때 들어가는 실수형 값은 Hz 단위를 사용합니다. 10 Hz는 $\frac{1}{10\text{Hz}} = 0.1$초 입니다.

앞서 언급한 대로 `ros::Duration`을 사용하기도 합니다.

```cpp
ros::Duration(1.0).sleep();
// or
ros::Duration d(0.5);
d.sleep();
```
`sleep()`은 다른 언어에서 사용되는 것처럼 선언된 시간 만큼 노드를 일시정지 시킵니다.

따라서 while문 안에서 publish와 같은 동작을 하는 경우 다음과 같이 `sleep()`을 사용하는 것이 좋습니다.

**Section 1~2**까지의 내용은 [링크](http://wiki.ros.org/roscpp/Overview/Time)에 자세히 설명되어 있습니다.

## Section 3. Spin

ROS는 **Spinning**이라는 개념을 사용하여 Subscribe의 메시지를 처리합니다. **Spinning**은 기본적으로 메시지가 도착할 때 콜백을 호출하는 것입니다. **Spinning**에는 아래와 같은 종류가 있습니다.

- `ros::spin()`: 노드가 종료될 때까지 콜백을 호출하는 루프로 들어갑니다.

- `ros::spinOnce()`: 도착한 모든 메시지에 대해 콜백을 호출하지만 코드를 차단하지는 않습니다. 이 함수를 `ros::Rate`와 함께 사용하여 새 메시지를 확인하는 빈도를 제어하는 경우가 많습니다.

- `ros::AsyncSpinner`: 여러 콜백을 병렬로 처리할 수 있습니다.

일반적인 스핀은 아래와 같습니다. 클래스에서 NodeHandle과 Publisher, Subscriber를 선언하여 사용하는 경우에도 자주 사용합니다.

```cpp
ros::spin();
```

아래와 같은 경우는 이전 예제에서 제공된 스크립트에서 보았는데, while문 안에서 사용하는 경우에는 `ros::spin()`을 쓰면 반복문이 더이상 돌지 않고 `spin`에 갇히게 되므로 `spinOnce()`를 사용하고 주기를 설정해줍니다.

```cpp
ros::Rate rate(10); // 10 Hz
while (ros::ok()) {
	ros::spinOnce();
	rate.sleep();
}
```
위에서 제시된 종류 중 `AsyncSpinner`는 조금 더 심화된 개념이므로 튜토리얼에서는 생략하였습니다.

관심이 있는 분들은 [링크](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)에서 콜백 큐에 대한 설명과 멀티쓰레딩을 이용한 **Spinning**에 대해서도 읽어보면 좋을 것 같습니다.
# 2.2 메세지에 대한 이해와 커스텀 메세지 다루기

## Section 1: Introduction to ROS Message

**1.1 Introduction**

ROS 메시지(msg)는 ROS 메시지의 필드를 설명하는 간단한 텍스트 파일입니다. 이 파일은 여러 언어로 된 메시지의 소스 코드를 생성하는 데 사용됩니다.

주로 `msg`경로를 패키지 경로에 만들어서 사용합니다.

우리가 앞선 노드 작성에서 사용하였던 `std_msgs/String` 타입은 다음 [링크](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)에서 정의를 살펴볼 수 있습니다.

```
string data
```

우리가 다음 실습에서 사용할 `geometry_msgs/Twist`라는 메세지는 아래와 같이 정의되어 있습니다. [링크](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)

```
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

이렇게 여러 개의 메세지 타입으로 조합된 메세지도 있는데, `geometry_msgs/Vector3`는 아래와 같이 정의되어 있습니다. [링크](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Vector3.html)

```
float64 x
float64 y
float64 z
```

따라서 `msg`는 단순하게 ROS에서 사용가능한 **토픽**의 자료형이라고 이해하면 됩니다.

**1.2 ROS Message Commands**

앞서 배운 `rostopic` 명령어와 마찬가지로 메세지를 다루는 `rosmsg` 명령어들이 있습니다.

ROS 환경에서 사용가능한 메세지 타입들을 보기위한 명령어입니다.

```bash
rosmsg list
```

특정 메세지의 구조나 정의를 볼 때는 아래 명령어를 사용합니다.

```bash
rosmsg show [message_type]
```
이러한 명령어들은 [링크](http://wiki.ros.org/rosmsg)를 참고합니다.

## Section 2: Creating a custom msg

**2.1 Define your message**

이제 필요에 맞게 새로운 메세지를 작성해보도록 하겠습니다.

이전 `publisher_node`와 `subscriber_node`는 `std_msgs/String` 메세지만 담고 있었지만 몇 번째 토픽인지는 알 수 없었습니다.

이번 섹션에서는 `publisher_node`에서 보낸 횟수와 문자열을 함께 보낼 수 있도록 새로운 메세지를 만들어 보겠습니다.

아래 명령어를 터미널에 입력합니다.
```bash
roscd im_newbie
mkdir msg
cd msg && touch Hello.msg
```
`roscd`는 이미 존재하는 패키지 경로로 갈 수 있습니다.
유사하게는 `cd $(rospack find im_newbie)`도 가능합니다.

이제 `msg`폴더를 만들고 `Hello.msg`라는 파일을 만들어서 열어줍니다.

거기에 아래와 같이 작성합니다.
```
int32 num
std_msgs/String message
```
일반적인 `string`도 사용가능하지만 ROS 메세지에 대해 조금 더 이해하고자 `std_msgs/String`을 사용하였습니다.

**2.2 Building .msg Files**

새로 작성한 `msg`를 사용하려면 `package.xml`과 `CMakeLists.txt`를 수정해준 후 빌드해야 합니다.

`package.xml`에는 다음 내용을 추가합니다.

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

`CMakeLists.txt`에는 아래 내용을 추가합니다.

```cmake
find_package(catkin REQUIRED
  # ...
  message_generation
)

# ...

catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)

# ...

add_message_files(
  FILES
  Hello.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

자세한 내용은 [링크](http://wiki.ros.org/msg)를 참고합니다.

**2.3 Modifying the Publisher and Subscriber Nodes**

Publisher 및 Subscriber 노드에서 사용자 지정 메시지를 사용하려면 다음과 같이 소스 파일을 수정합니다

`publisher_node.cpp`에서
- 작성한 custom 메세지를 포함합니다.:
  ```cpp
  #include <im_newbie/Hello.h>
  ```
- 메세지 타입 `im_newbie::Hello`을 생성합니다 (기존의 불필요한 내용은 지워줍니다):
  ```cpp
  im_newbie::Hello msg;
  msg.num = count;
  msg.message.data = "Hello, ROS!";
  ```
- `ros::Publisher pub`에 정의된 메세지 타입을 `<im_newbie::Hello>`로 바꿔줍니다.

- 메세지를 publish합니다:
  ```cpp
  pub.publish(msg);
  ```
- while문 마지막에 count를 증가시킵니다:
  ```cpp
  int count;
  while(ros::ok()) {
    // ...생략
    ++count;
  }
  ```

`subscriber_node.cpp`에서는:

- 마찬가지로 메세지 헤더파일을 추가합니다:
  ```cpp
  #include <im_newbie/Hello.h>
  ```
- callback 함수를 메세지 타입에 맞게 수정합니다:
  ```cpp
  void messageCallback(const im_newbie::Hello::ConstPtr& msg) {
    ROS_INFO("%d -th Received message: %s", msg->num, msg->message.data.c_str());
  }
  ```
작성이 완료되면 마찬가지로 빌드를 해줍니다.

`catkin build`는 간편하게 개별적인 패키지만 빌드하는 기능을 제공합니다. **im_newbie** 패키지 위치에서 `catkin build --this`를 해주면 빠르게 위치에 있는 패키지만 빌드합니다.

```bash
# ~/catkin_ws/src/im_newbie$
catkin build --this
```

빌드가 완료되면 이전처럼 각 노드들을 터미널에서 열고 실행해봅니다.
```bash
# Terminal 1
roscore
# Terminal 2
rosrun im_newbie publisher_node
# Terminal 3
rosrun im_newbie subscriber_node
```

**2.4 Add Dependencies**

이제 기존의 노드들은 만들어진 메세지를 의존성으로 가지게 됩니다.

지금은 `publisher_node`와 `subscriber_node`가 이전에 빌드가 되었고, 새로운 메세지를 그 후에 추가해서 빌드하였기 때문에 빌드 과정에서 별다른 에러가 나타나지 않았습니다.

하지만 새로운 메세지를 만듦과 동시에 노드를 작성하였다면 CMakeLists.txt를 아래와 같이 수정해주어야 합니다.

```cmake
add_executable(publisher_node src/publisher_node.cpp)
add_dependencies(publisher_node im_newbie_generate_messages_cpp)
target_link_libraries(publisher_node ${catkin_LIBRARIES})

add_executable(subscriber_node src/subscriber_node.cpp)
add_dependencies(subscriber_node im_newbie_generate_messages_cpp)
target_link_libraries(subscriber_node ${catkin_LIBRARIES})
```

우리가 `package.xml`에서 넣어준 `message_generation`에서 만들어지는 새로운 메세지 타입을 의존성으로 가진다고 선언해주는 것입니다.

아래는 우리가 새로 만들어준 메세지를 사용하는 cpp 스크립트 예시입니다.
```cpp
#include <im_newbie/Hello.h>
```

처음 각 노드들을 빌드하고자 할 때 마주치는 `include`문에서 컴파일러 입장에서는 처음 보는 헤더파일을 빌드해야 하므로 에러를 내게 됩니다.

위에서 수정해준 `CMakeLists.txt`에서는 각 노드에 의존성을 선언함으로써 이러한 문제를 해결해줍니다.
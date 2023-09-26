# 2.1 간단한 Publisher, Subscriber 노드 작성하기

## Section 1: A ROS Package
들어가기에 앞서 ROS 패키지를 만들어줍니다.

**1.1 Creating a Package**

이전에 만들어준 워크스페이스로 이동하고 패키지를 만들어줍니다.

```bash
cd ~/catkin_ws/src
# 빌드 명령어로 catkin_make 사용 시 catkin_create_pkg im_newbie std_msgs roscpp
# Introduction에서는 catkin build로 워크스페이스를 처음 빌드해두었으므로 아래 명령어 사용
catkin create pkg im_newbie --catkin-deps std_msgs roscpp
```
여기에서는 `std_msgs`과 `roscpp` 의존성만 추가해주었습니다. 패키지에 필요한 특정 의존성에 따라 명령어를 바꾸어줍니다.

**1.2 Adding Dependencies**

패키지가 다른 ROS 패키지에 의존하는 경우 `package.xml`에 해당 패키지를 선언해야 합니다.
자세한 사항은 ROS wiki의 [6. Customizing Your Package](https://wiki.ros.org/action/fullsearch/ROS/Tutorials/CreatingPackage#CustomizingYourPackage:~:text=rospack%0Aroslib%0Arospy-,Customizing%20Your%20Package,-This%20part%20of)를 참고합니다.


**1.3 Building the Package**

의존성까지 다 추가한 후에는 패키지를 빌드합니다

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash # 혹은 source ~/.bashrc, 빌드 후 다시 source를 해주어야 빌드한 내용이 적용됩니다.
```

자세한 빌드 명령어에 대해서는 [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)또는 [catkin build](https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html)를 참고합니다.

두 빌드 방식의 차이는 [링크](https://catkin-tools.readthedocs.io/en/latest/migration.html)를 참고합니다.

빌드가 에러없이 성공적으로 끝났다면 본격적으로 노드를 작성해보겠습니다.

---

## Section 2: Writing a Simple Publisher Node

**2.1 Creating the Source File**

아래 명령어를 통해 이동하고 파일을 만들어줍니다.

```bash
cd ~/catkin_ws/src/im_newbie/src
touch publisher_node.cpp
```

`touch`는 파일을 만들어줍니다. 코드를 작성할 때 사용하는 에디터로는 `nano, gedit, vim, code` 등이 있습니다.

!!! info
	code는 VSCode로 가장 널리 쓰이는 IDE입니다. GUI를 사용하지 못하는 경우에는 주로 nano, vim, vi 등을 사용합니다.

	```bash
	#필요한 경우 아래 명령어로 설치합니다.
	sudo apt-get install nano gedit vim code -y
	```

**2.2 Writing the Code**

에디터로 만들어준 `publisher_node.cpp` 파일을 열고 아래 코드를 추가해줍니다.

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
	/**
	 * ros::init() 함수는 다음을 수행하기 위해 argc 및 argv를 확인해야 합니다.
	 * 명령줄에서 제공된 모든 ROS 인자와 이름 리매핑을 수행할 수 있습니다.
	 * 프로그래밍 방식 리매핑의 경우 다른 버전의 init()을 사용할 수 있습니다.
	 * 리매핑을 직접 수행할 수 있지만, 대부분의 명령줄 프로그램에서는 argc와 argv를 전달하는 것이
	 * 전달하는 것이 가장 쉬운 방법입니다. init()의 세 번째 인수는 노드의 이름입니다.
	 *
	 * ROS 시스템을 사용하기 전에 ros::init()의 버전 중 하나를 호출해야 합니다.
	 */
	// "publisher_node"이름으로 노드 선언
	ros::init(argc, argv, "publisher_node");

	/**
	 * NodeHandle은 ROS 시스템과의 통신을 위한 주요 액세스 포인트입니다.
	 * 처음 생성된 NodeHandle이은 이 노드를 완전히 초기화하며, 마지막으로 생성된 NodeHandle이 소멸되면 노드가 닫힙니다.
	 */
	ros::NodeHandle nh;

	/**
	 * advertise() 함수는 주어진 토픽 이름에 대해 publish하고자 ROS에 선언하는 방법입니다.
	 * advertise()는 publish() 호출을 통해 해당 토픽에 메시지를 publish할 수 있는 publisher 객체를 반환합니다.
	 * advertise()의 두 번째 매개 변수는 메시지 publish에 사용되는 메시지 큐의 크기입니다. 메시지를 보낼 수 있는 속도보다 더 빨리 publish되는 경우 여기 숫자는 버리기 전에 버퍼링할 메시지 수를 지정합니다.
	*/
	// 앞서 선언한 NodeHandle nh의 advertise 함수로, "my_topic" 이름을 가지는 String 메세지 타입의 토픽을 publish할 수 있는 publisher를 선언
	ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);

	ros::Rate rate(1);  // 1 Hz

	while (ros::ok()) {
		std_msgs::String msg;
		msg.data = "Hello, ROS!";

		/**
		 * publish() 함수는 메시지를 보내는 방법입니다. 매개 변수는 메시지 객체입니다. 이 객체의 유형은 위의 생성자에서 수행한 것처럼 advertise<>() 호출에 템플릿 매개변수로 지정된 유형과 일치해야 합니다.
		 */
		// advertise에서 선언한 std_msgs::String에 해당하는 변수인 msg를 publish합니다.
		pub.publish(msg);

		// 1Hz로 publish 되도록 설정
		rate.sleep();
	}

	return 0;
}
```
위 설명의 대부분은 아래 링크의 내용을 번역한 것입니다.

자세한 내용은 [Writing a Simple Publisher and Subscriber (C++)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)을 참고합니다.

Publisher에 담긴 메시지나 코드를 작성하는 구체적인 방법에 대해서는 추후에 다루고 우선 기본적인 ROS의 노드가 무엇인지 이해하는 과정입니다.

**2.3 Building the Node**

다음 명령어를 통해 패키지와 Publisher 노드를 빌드합니다.

```bash
cd ~/catkin_ws
catkin build
```

이렇게 하면 빌드가 실패하며 에러가 나타날 것입니다.

처음 `catkin create pkg`를 통해 패키지를 생성하고 빌드하였을 때 우선 설명을 건너뛰었지만 파일 구조는 아래와 같습니다.

새로운 노드를 작성하거나 의존성이 추가되는 등등의 경우, `CMakeLists.txt`와 `package.xml`에 변경사항을 반영해주어야 빌드할 때 실패하지 않고 변경사항을 적용할 수 있습니다.
```
catkin_ws/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    im_newbie/
      CMakeLists.txt     -- CMakeLists.txt file for im_newbie
      package.xml        -- Package manifest for im_newbie
```

`package.xml`을 열어보면 처음 의존성에 추가해준 `roscpp`과 `std_msgs`가 아래와 같이 보일 것입니다.
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
```
여기에는 패키지 이름과 빌드 의존성, 그리고 패키지에 대한 설명을 작성할 수 있습니다.
원하는 경우 `<description>`, `<maintainer email>`이나 `<license>`등을 수정해도 됩니다.

위에서 넘어간 **1.2 Adding Dependencies** 링크에 설명되어 있습니다. 여기까지는 읽기만 하면 되는 내용들이고 이제 C++파일을 작성하였으므로 `CMakeLists.txt`는 직접 수정해주어야 합니다.

**im_newbie** 폴더 아래에 있는 `CMakeLists.txt`파일을 열고 아래 내용을 추가해줍니다.

```txt
add_executable(publisher_node src/publisher_node.cpp)
target_link_libraries(publisher_node ${catkin_LIBRARIES})
```
살펴본 두 파일 모두 주석으로 설명히 자세히 되어있고 기본적인 형식을 제공하므로 필요한 부분의 주석을 없애서 작성하여도 좋습니다.

이제 다시 아래 명령어를 실행해봅니다.

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```


**2.4 Running the Node**

다음 명령어를 통해 노드를 실행합니다.

앞으로의 명령어 예시에서는 `source /opt/ros/noetic/setup.bash`와 `source ~/catkin_ws/devel/setup.bash`를 `~/.bashrc`에 넣었다는 것을 가정하고 생략합니다.

넣지 않은 경우에는 각 명령어 실행 전에 두 파일을 모두 `source`해준 후에 실행하면 됩니다.
```bash
# Terminal 1
$ roscore

# Terminal 2
$ rosrun im_newbie publisher_node

# Terminal 3
$ rostopic echo /my_topic
```

ROS1에서는 MASTER 역할이 필요합니다. 추후 배울 `roslaunch` 커맨드에서는 마스터가 자동으로 포함되지만 여기서는 `rosrun`으로만 노드를 실행하므로 마스터를 켜주는 `roscore`를 입력해줍니다. ([공식문서링크](http://wiki.ros.org/roscore))

각 터미널로 구분한 명령어는 **터미널을 새로 열어 각 다른 터미널에서 입력**해야 합니다.

Termianl 2에서는 Publisher 노드가 실행 중이며 Terminal 3에서는 1Hz의 속도로 "Hello, ROS!" 메시지가 나오는 것을 확인할 수 있습니다.

`rostopic echo`는 원하는 토픽의 정보를 받아볼 수 있는 명령어입니다. `echo` 외에도 여러 명령어를 통해 실행되고 있는 토픽들의 상태와 정보를 알 수 있습니다.

자세한 내용은 [링크](http://wiki.ros.org/rostopic)에 설명되어 있고, 디버깅에 자주 사용하게 되니 추후 자연스레 익히면 됩니다.

---

## Section 3: Writing a Simple Subscriber Node

**3.1 Creating the Source File**

이제 `rostopic echo` 대신 노드를 통해 토픽을 subscribe 해봅니다.

```bash
cd ~/catkin_ws/src/im_newbie/src
touch subscriber_node.cpp
```

**3.2 Writing the Code**

에디터로 `subscriber_node.cpp` 파일을 열고 아래 코드를 입력합니다.

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void messageCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("Received message: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "subscriber_node");
	ros::NodeHandle nh;

	/**
	 * subscribe() 함수는 특정 토픽에 대한 메시지를 수신하는 방법입니다.
	 * 메시지는 콜백 함수에 전달되며, 여기서는 messageCallback()이라고 합니다.
	 * subscribe() 함수의 두 번째 매개 변수는 메시지 큐의 크기입니다. 메시지가 처리되는 속도보다 빨리 도착하는 경우 가장 오래된 메시지를 버리기 시작하기 전에 버퍼링되는 메시지 수입니다.
	 */
	// 앞서 선언한 NodeHandle nh의 subscribe() 함수를 이용하여 "my_topic" 토픽을 수신하고 이를 messageCallback() 함수에 넘겨줍니다.
	ros::Subscriber sub = nh.subscribe("my_topic", 10, messageCallback);

	/**
	 * ros::spin()은 콜백을 펌핑하는 루프에 들어갑니다. 이 버전에서는 모든 콜백이 이 스레드(메인 스레드) 내에서 호출됩니다.
	 * Ctrl-C를 누르거나 마스터에 의해 노드가 종료되면 ros::spin()이 종료됩니다.
	 */
	ros::spin();

	return 0;
}
```

**3.3 Building the Node**

**im_newbie** 폴더 아래에 있는 `CMakeLists.txt`파일을 열고 아래 내용을 추가해줍니다.

```txt
add_executable(subscriber_node src/subscriber_node.cpp)
target_link_libraries(subscriber_node ${catkin_LIBRARIES})
```

다음 명령어를 통해 패키지와 Subscriber 노드를 빌드합니다.

```bash
cd ~/catkin_ws
catkin build
```

**3.4 Running the Node**

위에서 작성한 Publisher 노드와 함께 Subscriber 노드를 실행합니다.


```bash
# Terminal 1
$ roscore

# Terminal 2
$ rosrun im_newbie publisher_node

# Terminal 3
$ rosrun im_newbie subscriber_node
```

이제 Subscriber 노드를 실행한 세 번째 터미널에서 `"my_topic"` 토픽을 수신하기 시작하고 수신된 메시지에 담긴 `data` 부분을 표시합니다.

위에서 작성한 `subscriber_node.cpp`에 보면 `printf`나 `std::cout`이 아닌 `ROS_INFO`를 통해 메세지를 출력하도록 하였습니다.

ROS에서 제공하는 터미널 출력 함수로 보통 `ROS_INFO`나 `ROS_WARN` 등을 통해 터미널 출력을 합니다. 자세한 내용은 [링크](http://wiki.ros.org/roscpp/Overview/Logging)를 참고합니다.

`printf`와 `ROS_INFO`와의 차이는 [ROS_INFO vs printf](https://answers.ros.org/question/12837/ros_info-vs-printf/) 에 좋은 답변이 있습니다.
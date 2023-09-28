# 3.2 직접 작성한 코드로 제어하기(예제 #1)

ROS의 개념들과 CLI를 익혔으니, 이전에 `turtle_teleop_key`를 통해 제어해본 것 대신 직접 코드를 작성해서 제어해보도록 하겠습니다.

## Section 1. Creating a Package

이전 챕터나 ros wiki를 참고하여 `roscpp`, `std_msgs`, `geometry_msgs` 의존성을 가지는 패키지를 하나 만드세요.

생성된 패키지의 `package.xml`을 적절하게 수정해주세요.

```bash
cd ~/catkin_ws
catkin build
```
혹은
```bash
catkin build --this # 패키지 위치에서
```
를 이용해 빌드해주세요.

## Section 2. Mission #1

이제 약간의 가이드 라인을 가지고 직접 소스코드를 작성해봅니다.

!!! info "예제 설명 및 조건"
    - 클래스 작성 없이 .cpp 파일 하나만을 이용해 노드를 작성합니다.
    - `main()`함수 안에 `ros::init`과 `ros::NodeHande`을 선언합니다.
    - 노드와 필요한 Publisher를 선언해주고 메세지 형식과 토픽 이름은 Chapter 3.1 혹은 [turtlesim 패키지 설명](http://wiki.ros.org/turtlesim)을 참고합니다.
    - 아래 주어지는 `getKey()`함수를 이용해 키보드 입력을 받습니다.
    - `while(ros::ok())` 반복문 안에 키보드 입력과 입력에 따른 조건문, 토픽 publish 내용을 넣습니다.
      ```cpp
      # 예시
      int input = getKey();
      if(input == 'w') {
        # do someting
      };
      # ...생략
      ```
    - `ros::Rate`를 이용해 반복문 주기를 설정하고 while문 마지막에 `sleep()`을 넣습니다.
    - `getKey()`를 사용하는 방식이 너무 어려운 경우, std::cin으로 필요한 값들을 한개 씩 입력받도록 합니다.
    - std::cin으로 각 값을 입력받는 경우 키보드로 제어하는 대신 필요한 입력값을 받아 publish하도록 합니다.
      ```cpp
      # 예시
      std_msgs::String msg;
      std::cin >> msg.data;
      ```

!!! example "getKey()"
    ```cpp
    #include <stdio.h>
    #include <termios.h>
    #include <unistd.h>

    int getKey(void)
    {
      int ch;
      struct termios oldt;
      struct termios newt;

      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;

      newt.c_lflag &= ~(ICANON | ECHO);
      newt.c_iflag |= IGNBRK;
      newt.c_iflag &= ~(INLCR  | ICRNL | IXON  | IXOFF);
      newt.c_lflag &= ~(ICANON | ECHO  | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
      newt.c_cc[VMIN] = 1;
      newt.c_cc[VTIME] = 0;
      tcsetattr(fileno(stdin), TCSANOW, &newt);

      ch = getchar();

      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

      return ch;
    }
    ```

추가로, 아래와 같은 터미널 안내문을 넣어주어도 좋습니다.

!!! example "printInfo()"
    ```cpp
    void printInfo()
    {
      puts("Remote Control turtle of turtlesim_node");
      puts("---------------------------------------");
      puts("               (forward)               ");
      puts("                   w                   ");
      puts("  (turn-left) a    s    d (turn-right) ");
      puts("                (back)                 ");
      puts("---------------------------------------");
      puts("### type Ctrl-C to quit                ");
      puts("");
    }
    ```

소스코드 작성이 완료되면 `CMakeLists.txt`를 적절히 수정해줍니다. 이전 Chapter 2.1을 참고하세요.

목표는 teleop_key 노드처럼 키보드 `w` `a` `s` `d`로 거북이를 제어하거나 직접 넣어준 값으로 거북이를 제어할 수 있으면 됩니다.

!!! tip "ros::ok()란?"
    while문에 `ros::ok()`를 넣는 이유는 다음과 같은 상황에서 `ros::ok()`가 false를 반환하기 때문입니다.

    - `Ctrl-C`가 입력된 경우

    - 같은 이름을 가진 다른 노드가 생긴 경우

    - `ros::shutdown()`으로 노드를 종료시킨 경우

    - `ros::NodeHandle`이 소멸된 경우

    따라서 `ros::ok()`를 사용함으로써 while문을 안전하고 정상적으로 종료할 수 있습니다.
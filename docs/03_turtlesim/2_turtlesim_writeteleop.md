# 3.2 직접 작성한 코드로 제어하기(예제 #1 #2)

ROS의 개념들과 CLI를 익혔으니, 이전에 `turtle_teleop_key`를 통해 제어해본 것 대신 직접 코드를 작성해서 제어해보도록 하겠습니다.

## Section 1. Creating a Package

이전 챕터나 ros wiki를 참고하여 `roscpp`, `std_msgs`, `geometry_msgs` 그리고 `message_generation` 의존성을 가지는 패키지를 하나 만드세요.

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

- 클래스 작성 없이 .cpp 파일 하나만을 이용해 노드를 작성합니다.
- 아래 주어지는 `getKey()`함수를 이용해 키보드 입력을 받습니다. (termios 헤더 파일 필수)
- 노드와 필요한 Publisher를 선언해주고 메세지 형식과 토픽 이름은 Chapter 3.1 혹은 [turtlesim 패키지 설명](http://wiki.ros.org/turtlesim)을 참고합니다.
- `while(ros::ok())` 반복문 안에 키보드 입력과 입력에 따른 switch문, 토픽 publish 내용을 넣습니다.

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
- `getKey()`를 사용하는 방식이 너무 어려운 경우, std::cout으로 필요한 값들을 한개 씩 입력받도록 합니다.
- std::cout으로 각 값을 입력받는 경우 switch대신 필요한 값을 모두 입력받는 경우 publish하도록 합니다.
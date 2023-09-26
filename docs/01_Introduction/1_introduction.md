## 1.1 ROS Noetic 설치하기

튜토리얼을 시작하기에 앞서, Ubuntu 20.04에 ROS Noetic를 설치합니다.
자세한 사항은 [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 을 참고하기 바랍니다.

```bash
# 터미널을 열어 아래 명령어들을 실행합니다.

# curl이 없는 경우 설치해줍니다.
sudo apt install curl -y

# packages.ros.org로부터 패키지를 받도록 설정합니다.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 패키지 리스트를 최신화합니다.
sudo apt update

# ROS Noetic Desktop Full 버전을 설치합니다.
sudo apt install ros-noetic-desktop-full

# 이 외 버전은 링크를 참고합니다.
```

---

## 1.2 워크스페이스 설정 및 환경 구성

ROS 작업 영역은 ROS 프로젝트를 개발하고 빌드하는 곳입니다. 이 섹션에서는 ROS 워크스페이스 설정, 패키지 생성에 대한 설명을 다룹니다.

```bash
# 아래 커맨드를 통해 ROS Noetic을 source 해줍니다.
source /opt/ros/noetic/setup.bash

# 매번 터미널을 열 때마다 ROS Noetic을 사용하기 위해서는 위 커맨드를 실행해야 합니다.
# 편리함을 위해서는 터미널 실행 시 자동으로 source되는 ~/.bashrc에 넣어주어도 됩니다.
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 새 워크스페이스를 만듭니다.
mkdir -p ~/catkin_ws/src

# 워크스페이스로 이동하여 빌드해줍니다.
cd ~/catkin_ws/
catkin init
catkin build

# 빌드 이후에는 아래 커맨드를 통해 사용자 워크스페이스의 빌드내용을 source 해주어야 합니다.
source devel/setup.bash

# 이후 ROS 패키지를 빌드하고 만들기 위한 의존성 도구들을 설치합니다.
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools -y

# rosdep은 시스템 종속성이나 ROS 핵심 구성 요소를 실행하는 데 필요합니다.
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

`source`는 쉘 스크립트를 실행하는 명령어로 `/opt/ros/noetic/`경로에 있는 `setup.bash`파일을 실행시켜 해당 터미널에서 ROS 환경을 사용할 수 있도록 합니다.

`~/.bashrc`는 여러 종류의 `dotfiles` 중 하나로 터미널이 실행되었을 때 자동으로 적용됩니다. 매번 커맨드를 입력하기 번거로운 환경 설정의 경우 보통 `.bashrc`파일에 넣습니다.

위치는 본인 계정의 홈 디렉토리에 있으며
```bash
cd ~ # or cd
gedit ~/.bashrc # 설치되지 않은 경우 sudo apt install gedit (혹은 본인이 선호하는 에디터로 열기)
```
을 통해 직접 열어볼 수 있습니다.

!!! tip
	보통 ROS 시스템 경로와 워크스페이스를 모두 `source`해주어야 설치된 패키지나 ROS 환경을 인식하기 때문에 `~/.bashrc`파일에는 아래와 같은 내용을 추가해줍니다.

	```bash
	source /opt/ros/noetic/setup.bash
	source ~/catkin_ws/devel/setup.bash # 워크스페이스를 catkin_ws 이름으로 홈 디렉토리에 만든 경우
	```

본인이 별도의 ROS 워크스페이스를 만들었거나 여러 개를 사용하는 경우
```bash
source <폴더 위치>/devel/setup.bash
```
를 해주어야 합니다.

---

## 1.3 shell 파일로 ROS Noetic 설치하기


현재 폴더에 있는 ros_install_noetic.sh로 위의 과정없이 간편하게 설치할 수도 있습니다.

[링크](https://github.com/qboticslabs/ros_install_noetic)에서 catkin build에 필요한 tool 설치 추가
```bash
# 해당 파일을 다운로드하여 터미널을 열고 아래 명령어를 입력
./ros_install_noetic.sh
```
shell 파일을 열어보고 .sh 형식의 파일이 어떤 기능을 할 수 있는지와 위 1.1 ~ 1.2에서 진행한 내용이 어떻게 담겨있는지 확인해봅니다.
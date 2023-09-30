# A ROS Noetic Tutorial for Beginners

![Title](./assets/banner_test_yonseidrone.png)

### Usage(for Administrator)
```bash
sudo python -m pip install mkdocs mkdocs-material pymdown-extensions
# or sudo pip install -r requirements.txt

# After editing...
mkdocs serve # Check the modifications on local site

git clone <site-url>
git add .
git commit -m "<commit-message>"
git push # or mkdocs gh-deploy
```
Github Action이 설정되어 있어 push만으로 새로 배포가 되지만, 그렇지 않은 경우에는 `mkdocs gh-deploy` 명령어를 사용합니다.

### TODO List

4. **PX4와 MAVROS**

	- [ ] : 4.1 PX4 Autopilot 설치

	- [ ] : 4.2 Gazebo 기초

	- [ ] : 4.3 원하는 모델과 월드 제작하기

	- [ ] : 4.4 Simulation

	- [ ] : 4.5 MAVROS
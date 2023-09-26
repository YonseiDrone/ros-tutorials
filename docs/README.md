# A ROS Noetic Tutorial for Beginners

![Title](./assets/banner_test_yonseidrone.png)

### Usage(for Administrator)
```bash
sudo python -m pip install mkdocs mkodcs-material pymdown-extensions
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
- [ ] : `ros::ok()` 설명 추가 [#](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
- [ ] : turtlesim을 이용해 rqt 사용하기
- [ ] : tf와 turtlesim 이용해 rviz 사용하기
# 2025

ROS2 Python 패키지를 사용합니다.
작년에 사용한 대회 코드들과(일부 수정됨) 카메라 테스트에 쓸만한 코드들입니다.
- GPT향 97.3% 함유
- GEMINI향 0.2% 첨가



# 구조 설명
현재 보이는 두 폴더는 워크스페이스 내의 src 폴더 안에 들어갈 패키지
```sh
cd ~/<워크스페이스>/src
git clone --no-checkout https://github.com/RnD-king/vision_sample.git
ls
```
> 예상 결과) my_cv robot_msgs

<워크스페이스>에 각자 워크스페이스 이름 넣고 위를 터미널에서 실행하면,
vision_sample 폴더 안에 my_cv, robot_msgs 폴더 2개가 보일 것

그 두 폴더는 패키지이니 vision_sample 폴더에서 빼서 src 폴더 안에 넣어주자. 그리고 vision_sample 폴더는 삭제.
완료했다면 <워크스페이스>/src 안에 정상적으로 패키지 2개가 설치된 것


## my_cv
영상처리 관련 실행 파일들이 들어있는 python 패키지.

> 패키지 이름을 내 맘대로 바꾸고 싶다면?
1. /my_cv/package.xml 파일에 들어간다
2. <name>my_cv</name> 에서 "my_cv"를 내가 원하는 이름의 패키지 이름으로 바꿔준다
3. 같은 패키지 폴더 안의 setup.py 파일에 들어간다
4. package_name = 'my_cv' 에서 "my_cv"를 동일한 이름으로 바꿔준다

# robot_msgs
ros2 토픽, 서비스 등에 사용되는 메세지들을 정의하는 패키지

자세한 건 핑크랩,,,

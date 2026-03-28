# 2025

ROS2 Python 패키지를 사용합니다.
<br>
작년에 사용한 대회 코드들과(일부 수정됨) 카메라 테스트에 쓸만한 코드들입니다.
- GPT향 97.3% 함유
- GEMINI향 0.2% 첨가



# 구조 설명
현재 보이는 두 폴더(my_cv, robots_msgs)는 패기지로,
<br>
워크스페이스 안의 src 폴더에 들어가야 한다.
<br>
<br>
설치법
```sh
cd ~/<워크스페이스>/src
git clone https://github.com/RnD-king/vision_sample.git
ls
```
> 예상 결과) <사용자이름>:~/<워크스페이스>/src/vision_sample$ my_cv robot_msgs
<br>
> 참고)
```
git clone <레포 URL>
```
> 터미널에 입력하면 깃헙 레포를 현재 위치에 설치할수 있다.
git이 없다고 터미널에 오류가 뜨면 오류 메세지를 따라 git을 설치하면 된다.
git clone을 알아두면 이것저것 설치하기 편하다
<br>
<br>
<워크스페이스>에 각자 워크스페이스 이름 넣고 위를 터미널에서 실행하면,
vision_sample 폴더 안에 my_cv, robot_msgs 폴더 2개가 보일 것

그 두 폴더는 패키지이니 vision_sample 폴더에서 빼서 상위 폴더인 src에 넣어주자. 그리고 vision_sample 폴더는 삭제.
완료했다면 <워크스페이스>/src 안에 정상적으로 패키지 2개가 설치된 것


# my_cv
영상처리 관련 실행 파일들이 들어있는 python 패키지.

> 패키지 이름을 내 맘대로 바꾸고 싶다면?
1-1. /my_cv/package.xml 파일에 들어간다
1-2. <name>my_cv</name> 에서 "my_cv"를 내가 원하는 이름의 패키지 이름으로 바꿔준다
2-1. 같은 패키지 안의 setup.py 파일에 들어간다
2-2. package_name = 'my_cv' 에서 "my_cv"를 package.xml과 동일한 패키지 이름으로 바꿔준다

## launch
여러 노드(주로 실행파일)들을 한 번에 실행하고, 설정까지 할 수 있는 파일을 모아놓은 폴더.
예)
```
ros2 run pkg node1
ros2 run pkg node2
ros2 run pkg node3
```
위와 같은 3개의 노드(실행파일)를 실행하려면 터미널도 3개의 창을 열어야 하고, 일일이 작성하기가 너무 귀찮으니
launch.py 파일 하나 만들어서 
```
ros2 launch my_package my_launch.py
```
처럼 사용하면, 쓰기도 쉽고 관리하기도 편해진다. (주의: launch 파일은 뒤에 .py까지 붙여야 함)

우리가 항상 사용하는 ros2 launch realsense_camera rs_launch.py도 이와 같음.

## src
실제로 사용하는 실행 코드 파일을 모아 놓은 폴더.

#### \_\_init\_\_.py
python 패키지의 src 폴더에 필수적으로 들어가야하는 파일.
내용은 아무것도 없지만 패키지가 정상 작동하려면 일단 필요하다.
실수로 없앴다면 그냥 빈 파일 만들고 이름을 \_\_init\_\_.py으로 붙이기만 해주자.

### 기능용 파일 모음
 

#### color_mask_test.py
내가 설정한 hsv 색공간 범위의 픽셀만 볼 수 있는 코드.

다른 터미널에 rqt를 실행시켜서 파라미터를 조절하면, 실시간으로 색범위를 조정할 수도 있으니
만약 주황색만 걸러내고 싶다면 파라미터를 조정해가며 내 카메라에서 주황색이 어떤 H,S,V값을 가지는지 확인할 수 있다. 
추가로, 출력 화면을 클릭하면 클릭한 부분의 HSV값도 터미널로 띄워주니 그 값에 맞춰도 좋다.

깊이 범위도 확인할 수 있어, 그냥 파라미터 튜닝할 때 안성맞춤인 코드.

#### depth_test.py
간단하게 내가 설정한 깊이 범위의 픽셀만 볼 수 있는 코드.
이또한 rqt를 사용하여 실시간 조정이 가능하니 depth 센서만 테스트하고 싶을 때 사용하면 된다.

코드가 짧고 파라미터 사용법이 나와있으니 공부해보면 좋다.

#### image_saver_roi_keypress.py
추후에 YOLO를 사용하게 되면, 우리는 YOLO 모델에 커스텀 데이터를 학습시켜줘야 한다.
데이터가 없는 YOLO는 갓 태어난 새끼 오리와 같기에, 우리는 YOLO에게 사진을 보여주며
"자~ 이게 점선이야~" 하고 5000번은 알려줘야 한다.
이 때, 각각 다른 사진이 필요하다. (작년에는 10000장을 사용했다)

또한 데이터의 통일성을 위해 리얼센스로 입력되는 화면 그대로를 사용해야 하므로
리얼센스 화면을 /<워크스페이스>/dataset/images/train 폴더 안에 저장해주는 이 코드를 사용하여 사진을 찍자.

참고로 찍어놓은 동영상을 단위 시간마다 나누어서 캡처해주는 프로그램도 있다.
노가다가 귀찮다면 이 방법을 찾아보는 것도 방법.

그렇게 사진 찍기가 모두 끝났다면, 라벨링 지옥이 기다리고 있으니 각오하자\,\,\,

#### realsense_test.py
리얼센스를 노드로 실행해보는 아주 기본적인 코드.
열 번 봐도 부족함이 없으니 열심히 익혀보자.

### 영상처리 코드들
\* 개인적인 훑어보기 추천 순서 : line_publisher > line_subscriber > ball_detect > ball_recieve
\* 나머지는 살짝만 훑어보기로? 듀얼 카메라를 사용한다면 ball_and_hoop를 참고해봐 좋지만, 분명 이 코드보단 좋은 방법이 있을 거라 생각한다.

#### line_publisher.py
화면 보정을 하고, 점선을 탐지하고, 점선 좌표들을 리스트 모아 퍼블리시하는 코드.

class RectangleTracker: 같은 점선을 가만히 보고 있어도 1프레임 정도는 중간에 끊기는 일이 있을 수 있다. (노이즈나 카메라 흔들림 등의 이유로)
혹은 같은 이유로, 바닥을 보고 있더라도 1프레임 정도는 점선으로 인식하는 일이 발생할 수도 있다. 
이렇게 치명적으로 작용할 수도 있는 간헐적 환각을 최대한 줄이기 위해 넣은 장치라 생각하면 된다.

이미 점선이라 인식하던 객체는 잠시 놓쳐도 그 좌표를 기억해두고, 새로 점선이라 인식하게 된 객체는 몇 프레임 정도는 연속으로 인식해야만 점선으로 인정받게 된다.
따라서 하나의 점선마다 5개의 정보를 갖고 있는데,
1. ID -점선의 고유 번호. ID가 다르면 다른 점선임
2. cx - 점선의 x 좌표
3. cy - 점선의 y 좌표
4. lost - 이 점선을 연속으로 놓친 지 몇 프레임이 지났는지. self.lost보다 커지는 순간 이 점선은 리스트에서 지워버린다.
5. found - 이 점선을 연속으로 몇 번 탐지했는지. self.found보다 커지는 순간 진짜 점선으로 인식되어 점선 리스트 등록한다.

위 정보들을 통해, 각 프레임마다의 점선 정보를 저장, 다음 프레임으로 업데이트한다.
물론 로봇이 이동하면서 점선의 위치도 이동하게 될텐데, 각 프레임마다 max_dist 이하만큼 움직인 점선만이 같은 점선이라 인식된다.
조금 rough한 방법이긴 하지만, 가볍고 단순하게 구현하기엔 이게 제일 나아서...
고급과정) 칼만필터, 옵티컬 플로우

class ImgSubscriber(Node): 익숙한 클래스일 것이다. 콜백함수를 내장하고 있는 노드 객체.
```
        self.subscription_color = self.create_subscription(  # 컬러
            Image,
            '/camera/color/image_raw',  # RealSense에서 제공하는 컬러 이미지 토픽  >>  640x480 / 15fps
            self.color_image_callback, 10)
        
        self.line_publisher = self.create_publisher(
            LinePointsArray, 
            'candidates', 
            10) # 중심점 퍼블리싱
```
각각 realsense 이미지를 구독하는 구독자, 점선의 중심점을 발행할 발행자를 만든다.
점선의 중심점은 LinePointsArray라는 메세 타입으로 보내짐. (robots_msgs 참고)

```
# 파라미터 선언
self.declare_parameter("max_lost", 10)
...
self.declare_parameter("rect_area_min", 200)
```
노드를 실행하는 중간에도 rqt를 통해 색 보정치나 점선 조건 등을 변경할 수 있게 만드는 파라미터를 선언한다.
("이름", 값)
```
 # 파라미터 적용
        max_lost = self.get_parameter("max_lost").value
        ...
        self.rect_area_min = self.get_parameter("rect_area_min").value
```
선언한 파라미터를 이 코드에서 사용할 변수 이름으로 적용한다.
```
self.add_on_set_parameters_callback(self.parameter_callback)
```
이후 파라미터가 rqt에서 변경될 때마다 parameter_callback 함수가 실행되어 노드에 전달해줄 예정.
```
def parameter_callback(self, params):
```
여기서 각 파라미터의 값들의 범위를 지정해줄 수 있다.
사실 없어도 돌아가긴 하지만, 유효한 값 범위를 확실히 주고 싶으니 사용했다.
```
for param in params:
    return SetParametersResult(successful=True)
```
이것만 있어도 된다. (대충 파라미터 값 적용하겠다는 뜻)

```
def get_angle
```
그냥 각도 계산 함수. 별 뜻 없음
```
def color_image_callback(self, msg):
```
이 코드의 본체.
리얼센스에서 초당 15번 이미지 데이터를 보낼 때마다 실행되는 콜백함수.
자세한 건 주석 참고

```

def main():
    rp.init()
    node = ImgSubscriber()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
```

그냥 외우자.
ROS2 노드를 돌릴 때 사용하는 방식이다.

# robot_msgs
ros2 토픽, 서비스 등에 사용되는 메세지들을 정의하는 패키지

자세한 건 핑크랩,,,



realsense 카메라를 실행시키는 rs_launch.py가 들어있는 launch 폴더와
여러 코드 실행 파일들이 들어있는 src 폴더,
이 패키지가 무엇이고, 무엇이 필요하고, 어떻게 빌드되는지 설명해주는 package.xml,
Python 패키지를 설치하고 등록하는 파일인 setup.py로 이루어짐

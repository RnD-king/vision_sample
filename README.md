# 2025 이토록 사소했던 영상처리
   
ROS2 Python 패키지를 사용합니다.

작년에 사용한 대회 코드들과(일부 수정됨) 카메라 테스트에 쓸만한 코드들입니다.
- GPT향 97.3% 함유
- GEMINI향 0.2% 첨가


***
# 구조 설명
두 개의 패키지로 이루어져 있습니다.(my_cv, robots_msgs)   

### * 패키지 설치법

패키지는 워크스페이스 안의 src 폴더에 들어있어야 하므로,
```sh
cd ~/<워크스페이스>/src
git clone https://github.com/RnD-king/vision_sample.git
ls
```
> 예상 결과) <사용자이름>:~/<워크스페이스>/src/vision_sample$ my_cv robot_msgs

<워크스페이스>에 각자 워크스페이스 이름 넣고 위를 터미널에서 실행하면,  
vision_sample 폴더 안에 my_cv, robot_msgs 폴더 2개가 보일 것이다.

그 두 폴더를 vision_sample 폴더에서 빼서 상위 폴더인 src에 넣어주자.  
그리고 vision_sample 폴더는 삭제.

완료했다면 <워크스페이스>/src 안에 정상적으로 패키지 2개가 설치된 것

__참고)__

```
git clone <레포 URL>
```
이를 터미널에 입력하면 주소에 맞는 깃헙 레포를 현재 폴더 위치에 내려받을 수 있다.

만약 실행했는데 git이 없다고 터미널에 오류가 뜨면, 오류 메세지를 따라 git을 설치하면 된다.  
git clone을 알아두면 이것저것 설치하기 편하니 알아두자.



***
# 패키지 설명
# 1. my_cv
영상처리 관련 실행 파일들이 들어있는 python 패키지.

> 패키지 이름을 내 맘대로 바꾸고 싶다면?

1-1. /my_cv/package.xml 파일에 들어간다  
1-2. "my_cv"를 모두 내가 원하는 이름의 패키지 이름으로 바꿔준다  

2-1. 같은 패키지 안의 setup\.py 파일에 들어간다  
2-2. "my_cv"를 모두 package.xml과 동일한 패키지 이름으로 바꿔준다  

3-1. 같은 패키지 안의 setup.cfg 파일에 들어간다  
3-2. 이하 동문  

4-1. 워크스페이스로 돌아가 build, install, log 폴더들을 지우고 colcon build를 해주자.  
(build, install, log 폴더는 임시파일이라 새로 빌드를 할 거면 지워도 지장이 없다)

## 1-1. src
실제로 사용하는 실행 코드 파일을 모아 놓은 폴더.

### **0) \_\_init\_\_.py**
python 패키지의 src 폴더에 필수적으로 들어가야하는 파일.  
내용은 아무것도 없지만 패키지가 정상 작동하려면 일단 필요하다.  
실수로 없앴다면 그냥 빈 파일 만들고 이름을 \_\_init\_\_.py으로 붙이기만 해주자.

### \=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\= 기능용 파일들 (알파벳 순서) \=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\= 

대회에 사용할 코드는 아니지만, 색보정이나 다양한 파라미터 테스트 or 카메라 테스트 등을 할 수 있는 코드들.  
단순하게 구성되어 있으니 추가로 원하는 기능을 넣어 본인만의 코드를 만들 수도 있을 것이다.  


### 1) color_mask_test.py
내가 설정한 hsv 색공간 범위의 색만 화면에 출력하는 코드.

만약 주황색만 걸러내고 싶다면, 먼저 구글에 hsv 색상표를 검색해서 주황색에 맞는 H,S,V 값들을 알아내자.  
그 후에 코드에서 그 값을 직접 넣거나, rqt의 Parameter Reconfigure에서 파라미터 값을 조정해가며  
카메라 화면에서 어떤 부분이 주황색인지 확인할 수 있다.

다만 코드에 값을 직접 넣는 방법은 코드 수정 및 재빌드가 필요하므로 귀찮으니 rqt를 애용하자.  

> rqt 사용법  
> 새 터미널 > source~ > rqt입력 > 상단 메뉴의 Plugins-Configuration-Dynamic Reconfigure 클릭  
> 왼쪽에서 color_test_node를 클릭한 후, 파라미터 값을 바꿔보자  
> 간혹 Dynamic Reconfigure를 클릭해도 아무것도 뜨지 않을 때가 있는데, 그러면 뜰 때까지 다시 눌러주자

추가로, 출력 화면을 마우스 클릭하면 그 클릭한 부분의 HSV값을 터미널로 띄워주니 그 값에 맞춰도 좋다.

\+ 깊이 범위도 다른 화면에서 확인할 수 있어, 그냥 파라미터 튜닝할 때 안성맞춤인 코드.  
대회장에서 쏠쏠하게 써먹었다.

### 2) depth_test.py
간단하게 내가 설정한 깊이(거리)만 볼 수 있는 코드.  
이 코드 또한 rqt를 사용하여 실시간 조정이 가능하니 depth 센서만 테스트하고 싶을 때 사용하면 된다.

코드가 짧고 파라미터 사용법이 나와있으니 공부해보면 좋다.  

### 3) image_saver_roi_keypress.py
추후에 YOLO를 사용하게 되면, 우리는 YOLO 모델에 커스텀 데이터를 학습시켜줘야 한다.

데이터가 없는 YOLO는 갓 태어난 새끼 오리와 같기에, 우리는 YOLO에게 사진을 일일이 보여주며  
"자~ 이게 점선이야~" 하고 수천 번은 알려줘야 한다.  
물론, 각각 다른 사진이 필요하다. (작년에는 점선 단일 class를 위해 2800장을 사용했다)

또한 데이터의 통일성을 위해 카메라에 입력되는 화면 그대로를 사용해야 하므로 리얼센스로 찍은 사진이 필요하다.  
(폰으로 찍고 옮기기 X. 카메라마다 색보정이 살짝씩 다르기에)

그래서 이 코드는 실행 시 스페이스바를 누르면 리얼센스로 출력되는 화면을  
~/<워크스페이스>/dataset/images/train 폴더 안에 저장해준다

이 코드가 아니더라도 찍어놓은 동영상을 단위 시간마다 나누어서 자동으로 사진으로 만들어주는 편리한 프로그램도 있다.  
노가다가 귀찮다면 이 방법을 찾아보는 것도 방법.

\+ 어떤 방법으로든 사진 찍기가 모두 끝났다면, literally 순수 노가다 라벨링 지옥이 기다리고 있으니 각오하자\,\,\,  

### 4) realsense_test.py
리얼센스를 ROS2 노드로 실행해보는 아주 기본적인 코드.  
열 번 봐도 부족함이 없으니 열심히 익혀보자.

### \=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\= 영상처리 코드들 \=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=\=
\* 추천하는 훑어보기 순서 : line_publisher > line_subscriber3 > line_subscriber > ball_detect > ball_recieve  
\* 나머지는 살짝 참고 정도만 하면 좋을 것 같다.  
\* 듀얼 카메라를 사용한다면 ball_and_hoop의 카메라 나누는 구성을 참고해봐도 좋지만, 분명! 이 코드보단 좋은 방법이 있을 거라 생각한다.

### 1) line_publisher.py
요약: 1. 화면 보정을 하고, 2. 점선을 탐지하고, 3. 점선 좌표들을 리스트 모아 퍼블리시하는 코드.

```python
class RectangleTracker:
```
__\= 점선 추적기__  
카메라가 같은 점선을 가만히 보고 있더라도 분명 가끔씩은 1\~2프레임 정도 인식을 못 하고 끊기는 일이 발생한다. 
(조건문의 문제일 수도 있고, 카메라 노이즈나 흔들림 등의 이유로)  
혹은, 점선이 없는 바닥을 보고 있더라도 1\~2프레임씩 노이즈를 점선으로 인식하는 일이 같은 이유로 발생한다.  
이러한 간헐적 환각을 최대한 줄이기 위해 넣은 장치라 생각하면 된다.

이미 점선이라 인식하던 객체는 잠시 놓쳐도 몇 프레임 정도는 원래의 좌표를 기억해두고,  
새로 점선이라 인식하게 된 객체는 몇 프레임 정도는 연속으로 인식해야만 점선으로 인정받게 된다.  
따라서 하나의 점선마다 5개의 정보를 갖고 있는데,

1. ID -점선의 고유 번호. ID가 다르면 다른 점선임  
2. cx - 점선의 x 좌표  
3. cy - 점선의 y 좌표  
4. lost - 이 점선을 몇 프레임 연속으로 놓쳤는지.  
   \- 임계값(max_lost) 초과 시: 이 점선은 tracking 중인 객체 리스트에서 제거됨
5. found - 이 점선을 몇 프레임 연속으로 탐지했는지.   
   \- 임계값(min_found) 이상 시: 불확실한 객체 → 확정된 점선(rectangle)으로 승격
   \- 이후로는 모든 정보가 rectangles 딕셔너리에 저장됨
> *rectangles = {
    0: (cx, cy, lost, found),
    1: (cx, cy, lost, found),
    2: (cx, cy, lost, found)
    ...
}*

위 정보들을 통해, 각 프레임마다 점선 정보를 저장, 다음 프레임으로 업데이트한다.

물론 로봇이 이동하면서 점선의 위치도 로봇 이동 방향과 반대로 카메라에서 이동하게 될텐데,  
이 때 각 프레임마다 max_dist 이하만큼 움직인 점선만이 같은 점선이라 판단한다.  
조금 rough한 방법이긴 하지만, 가볍고 단순하게 구현하기엔 이게 제일 나아서...  

고급과정) 칼만필터, 옵티컬 플로우

```python
class ImgSubscriber(Node):
```
익숙한 클래스일 것이다. 이미지 콜백함수를 내장하고 있는 노드 class.
```python
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
점선의 중심점은 LinePointsArray라는 메세지 타입으로 candidates라는 토픽을 퍼블리시. (robots_msgs 참고)

```python
# 파라미터 선언
self.declare_parameter("max_lost", 10)
...
self.declare_parameter("rect_area_min", 200)
```

노드를 실행하는 중간에도 rqt를 통해 색 보정치나 점선 조건 등을 변경할 수 있게 만드는 파라미터를 선언한다.  
self.declare_parameter("이름", 값)
```python
 # 파라미터 적용
        max_lost = self.get_parameter("max_lost").value
        ...
        self.rect_area_min = self.get_parameter("rect_area_min").value
```
선언한 파라미터를 이 코드에서 사용할 변수 이름으로 적용한다.
```python
self.add_on_set_parameters_callback(self.parameter_callback)
```
이후 파라미터가 rqt에서 변경될 때마다 parameter_callback 함수가 실행되어 노드에 전달해줄 예정.


```python
def parameter_callback(self, params):
```
파라미터가 변경될 때마다 실행되는 콜백함수.  
설정한 파라미터 값을 리턴한다. (=변경된 파라미터 값을 적용할 함수)

추가로, 여기서 각 파라미터의 값들의 범위도 지정해줄 수 있다.  
유효한 값 범위 내에서만 적용되게 만들고 싶다면 지금처럼 if, else문의 향연이 필요하지만,
```python
def parameter_callback(self, params):
   for param in params:
       return SetParametersResult(successful=True)
```
귀찮다면 다 필요없고 그냥 이렇게만 있어도 실행은 된다.

```python
def get_angle()
```

그냥 각도 계산 함수. 중요 x

```python
def color_image_callback(self, msg):
```
이 코드의 본체이다.  
리얼센스에서 초당 15번 이미지 데이터를 보낼 때마다 실행되는 콜백함수로,  
자세한 건 파일 내 주석 참고

```python
def main():
    rp.init()
    node = ImgSubscriber()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
```

마지막은 그냥 외우자.  
ROS2 노드를 python으로 돌릴 때 사용하는 방식이다.

### 2) line_subscriber3
요약: line_publisher에서 발행한 점 좌표를 받아오고, 그 데이터를 터미널에 단순 출력하는 코드

subscriber에 대한 아주아주 기본적인 코드다.  
자주 보고 눈에 익혀두자.
```python
import rclpy
from rclpy.node import Node
from robot_msgs.msg import LinePointsArray  # type: ignore
```
필요한 거 import 하기.  
뒤에 # type: ignore은 가끔 vscode가 커스텀 메세지를 인식 못 할 때가 있어서 빨간줄 긋지 말라고 해놓는 것.  
없어도 전혀 지장 없다.
```python
class LineSubscriberNode(Node):
    def __init__(self):
        super().__init__('line_subscriber3')
        self.sub = self.create_subscription(
            LinePointsArray,
            'candidates',
            self.line_callback,
            10
        )
```
LinePointArray라는 메세지 타입에서 'candidates'라는 토픽을 받을 거고,  
토픽을 받으면 self.line_callback라는 콜백 함수를 실행하겠다는 뜻.  
당연히 publisher 쪽이랑 메세지 이름, 토픽 이름이 같아야한다.
```python
    def line_callback(self, msg: LinePointsArray):
        if not msg.points:
            return
```
퍼블리셔 쪽에서 데이터 없으면 보내지도 않긴 하지만, 혹여나 0 데이터가 들어올 수도 있으니,  
만약 토픽 내용이 없으면 출력하지 말고 바로 return하여 끝낸다
```python
        for idx, point in enumerate(msg.points):
            self.get_logger().info(
                f'[{idx}] cx={point.cx}, cy={point.cy}, lost={point.lost}'
            )
```
데이터가 들어오면 순서대로 ID, cx, cy, lost를 출력하는 코드.  
딕셔너리라서 for idx, point in enumerate(msg.points):라고 쓴다.

### 3) line_subscriber

방금 line_subscriber3.py 에서 본 중심 좌표만 받아오기(출력하기)에 이것저것 추가된 코드  

요약: 중심점 좌표 받아오기 > 좌표들의 분포를 통해 정해둔 3가지 기준에 따라 로봇의 행동 결정 > 결정된 행동을 퍼블리시 

```python
camera_width = 640 
camera_height = 480 # 해상도

roi_x_start = int(camera_width * 0 // 5)
roi_x_end = int(camera_width * 5 // 5)
roi_y_start = int(camera_height * 1 // 12)
roi_y_end = int(camera_height * 11 // 12)
# 전체 카메라 화면에서 연산 속도 효율을 위해, 실제로 보정, 추론 등은 ROI 내부에서만 진행하도록 화면 일부만 사용함.
# 이 숫자들을 적용한다면, 화면의 세로 위아래 1/12씩은 이미지를 받아올 뿐, 전처리나 추론은 전혀 진행되지 않는다.

zandi_x = int((roi_x_start + roi_x_end) / 2)
zandi_y = int((roi_y_start + roi_y_end) / 2) + 140 # 카메라 좌표 상에서 로봇이 서있는 위치(원점 좌표처럼 사용)
```
코드 전체에서 사용할 상수들을 정의했다. 카메라 크기 같은 거

```python
class LineListenerNode(Node): #################################################################### 판단 프레임 수 바꿀 때 yolo_cpp 도 고려해라~~~
    def __init__(self):
    ...
```
여기부터 자세한 내용은 주석 참고 바람.

다만, 코드를 내리다보면 밑에 처음보는 함수가 있을 것이다.
```python
    def motion_callback(self, msg: MotionEnd): # 모션 끝 같이 받아오기 (중복 방지)
        if bool(msg.motion_end_detect):
            self.armed = True
            self.get_logger().info("Subscribed /motion_end !!!!!!!!!!!!!!!!!!!!!!!!")
```
이름에서 알 수 있듯, 이 함수도 콜백함수이다.  
그렇다면, line_publisher의 토픽을 받아오는 콜백도 아니고, realsense의 이미지 토픽을 받아오는 콜백도 아니고, 무엇을 위한 콜백일까?  
바로 모션 쪽의 토픽을 받아오는 함수이다.

지금 이 코드는 작년 대회에 사용했던 코드와 거의 유사하다(YOLO 내용이 들어있지 않는 것 정도의 차이?)

대회에서 로봇은 가만히 서서 바닥을 15프레임(1초) 관찰하고, 그 관찰한 결과로부터 다음 동작을 결정했다.  
다음 동작이 끝나면 멈춰서 다시 바닥을 관찰하고\,\,\, 그런 식.

그렇다면 동작이 끝났다는 것을 영상처리 쪽에서 인지를 해야만, 그 뒤에 바닥을 관찰하며 탐지 코드(line_publisher, line_subscriber 등)를 돌려야 할 것이다.  
때문에 로봇이 동작을 모두 끝냈다면, 모션을 담당하는 파트에서 motion_end_detect 라는 이름의 토픽으로 True값을 한 번 보낸다.  
그 때 이 모션 콜백 함수가 그 토픽을 받으면서 실행되어 탐지 코드를 시작할 수 있게 만들어 준다.

처음부터 살펴보자

우선, 이 코드엔 2가지 상태변수가 있다.  
1. self.collecting  
2. self.armed

둘 다 motion_end가 들어온 뒤에 실행되기 위해 필요한 일종의 차단기와 같은 역할을 하는 상태변수로, 기본값은 모두 False이다.  
상태 변수의 의미:
- self.armed = False → 준비 안 됨 (motion_end 대기 중)
- self.armed = True → 준비 완료 (motion_end 받음, 다음 line_callback에서 collecting=True로 전환)
- self.collecting = False → 데이터 수집 안 함 (15프레임 루프 미실행)
- self.collecting = True → 데이터 수집 중 (15프레임 동안 점선 분석)

line_callback 함수를 이 두 상태변수 관점에서 크게 본다면, 
```python
def line_callback():
    if not self.collecting:
        if not self.armed:
            return:
        else:(코드에선 생략됨. 어차피 위에 return이 있어서 if에 걸리는 조건이면 바로 끝나기 때문)
            self.collecting = True
            self.armed = False
            <이하 몇몇 변수들 초기화>
    else:(코드에선 생략됨. 이유는 아래서 설명 예정)
        <이하 본격 line_subscriber 기능들>
        ...
```
이다.

따라서 평소에는 두 변수 모두 False 이므로, 모든 루프에서 
```python
    if not self.collecting:
        if not self.armed:
```
을 충족해 항상 return하며 아무 행동도 하지 않고 끝이 난다.

그런데 이 때, 모션 토픽이 들어와 motion_callback이 실행되면 (if bool(msg.motion_end_detect) 조건을 만족하므로)  
```python
            self.armed = True
            self.get_logger().info("Subscribed /motion_end !!!!!!!!!!!!!!!!!!!!!!!!")
```
motion_callback함수에선 armed를 True로 만들고, 
```python
else:(코드에선 생략됨. 어차피 위에 return이 있어서 if에 걸리는 조건이면 바로 끝나기 때문)
            self.collecting = True
            self.armed = False
            <이하 본격 line_subscriber 기능들>
            ...
```
line_callback함수에선 else문으로 넘어온다.  
이곳에서 armed는 collecting을 True로 만들고 본인은 다시 False가 된다.  
line_callback은 15프레임 동안 점선을 탐지하다가 끝나고, 다음에 다시 새로운 15프레임 동안 점선을 탐지하는 사이클을 갖기에,  
각 사이클마다 새로 초기화되거나 갱신되는 요소가 있어야 한다.(디버깅용 타이머나 함수 안에서 사용되는 몇몇 변수들 등)  
따라서, 새로운 15프레임 동안 line_callback이 잘 돌아가기 위해 이것저것 세팅하는 과정이라 생각하면 편하다.

그렇게 마지막 줄인,
```python
self.get_logger().info(f'[Start] Window {self.window_id} | I got {self.collecting_frames} frames')
```
까지 실행했다면, 바로 아래 줄로 내려갈텐데, 이곳엔 if not self.collecting:에 대한 else문이 없다.  
이유는 직전에 준비과정을 끝냈으니 그냥 바로 새로운 프레임을 받아와도 되기 때문.  
그러므로 코드 진행상, 준비과정이 끝나자마자 그 아래의 줄인
```python
# 진짜
        self.candidates = new_candidates
```
부터 실행되어 본격적인 line_subscriber가 실행한다.

클로드가 요약해준 타임라인:
1. [초기] armed=False, collecting=False
2. [motion_end 수신] armed=True (motion_callback 실행)
3. [첫 line_callback] armed→False, collecting→True (준비 시작)
4. [15프레임 루프] collecting=True 상태에서 계속 분석
5. [결과 발행] 15프레임 후 collecting→False (루프 종료)
6. [다시 대기] armed=False, collecting=False로 돌아가 다음 스텝 대기

이로써 우리의 line_subscriber.py는 motion_end_detect 토픽과, candidates 토픽 모두가 들어와야만 실행되는 구조를 갖추었다.

그렇다면 들 수 있는 의문:  
어차피 line_subscriber의 메인 실행 함수인 line_callback 함수는 LinePointsArray 메세지가 와야만 실행이 될텐데,  
방금과 같은 motion_end_detect 감지 로직은 이곳이 아니라 line_publisher에 있어야 하는 것 아닐까?

답:  
맞는 말이다. 굳~이 line_subscriber에까지 이런 로직이 있을 필요는 없다.  
그리고 현재 line_publisher에는 이 로직이 구현되지 않았지만, 실제론 구현되어있어야 한다.  
현재 publisher 코드는 단순화한 버전이기 때문에 넣지 않았을 뿐이다.

그럼에도 굳이굳이 subscriber에도 이 로직을 넣은 이유는,  
정말 만에 하나, 혹시라도 LinePointsArray 메세지가 타이밍이 꼬여서 잘못 들어오는 불상사가 발생하더라도 막아줄 수 있는 안전장치이기 때문.  
코딩에 안전장치는 성능을 해치지 않을 정도라면 많을 수록 좋다!

아무튼 이제 본격적으로 line_callback이 실행된다면, candidates로부터 동작 결정을 위한 판단 로직이 실행된다.  
점선 좌표로부터의 다음 동작 판단 기준을 대충 설명하자면,  
1. 현재 코스의 기울기가 얼마인가 (self.tilt_text)  
    중심점 좌표들을 1차식으로 근사한 후, 직선의 기울기를 구한다. 기울기가 클 수록 코스에 비뚤어진 채로 서있다고 판단  
2. 현재 코스가 직선구간인가 곡선구간인가 (self.curve_text)  
    근사한 직선과 가장 위쪽의 점선 중심점 사이의 거리를 구하고, 일정 거리 이상 떨어져있다면 현재 코스가 곡선 구간이라 판단.  
3. 점선들로부터 로봇이 얼마나 떨어져있는가 (self.out_text)  
    근사한 직선과 로봇의 중심점 사이의 거리를 구하고, 그 거리가 클 수록 코스로부터 로봇이 멀리 벗어나있다고 판단.

이 3가지 판단 기준으로부터, 다음 어떤 동작을 실행했을 때 로봇이 점선 코스 위에 서있고,  
방향도 똑바로 볼 수 있을지를 계산해서 다음 동작을 결정한다. (status)  
ex) status = 1 이면 전진, status = 2 면 좌회전 하면서 전진... 이런 식 

153줄부터 359줄까지 이런 내용이다... (경우의 수가 많아서 긴 것뿐 절대 어려운 로직이 들어간 건 아니다. 단순 노가다. 여러분은 이런 거 하지마라...)  
ex) 기울기: straight / 직선or곡선: 왼쪽으로 곡선 구간 / out: 오른쪽으로 out  일 때 status는 몇이고..

그 후에 15프레임 동안 line_callback이 실행되며 15개의 동작을 결정이 모였을 때,  
그동안 보고 내린 동작(status) 결정 중 가장 많이 나온 동작(res)을 최종 선택하여 퍼블리시한다.  
*status와 res는 둘 다 동작 명령을 의미하는 변수니 혼동 주의*  
*매 프레임의 결과로 선택한 동작이 status, 그 status를 15프레임 동안 보고 종합해서 최종 결정한 status 값이 res*

그리고 그 동작(res)은 알고리즘 쪽의 코드에서 구독하여 사용한다.  
> (참고: line, ball_and_hoop, hurdle 세 코드 모두 한 번의 최종 결정 루프마다 하나의 res를 발행한다  
> 따라서 알고리즘은 한 루프마다 3개의 res를 받게 되는데, 각 코드별 가중치에 따라 하나의 res를 최종 결정한다  
> 예) 점선도 탐지했고, 농구공도 탐지했다면, 농구공으로 걸어가는 것이 우선이기 때문에)

나머지 자세한 내용은 주석 참고.


```python
    def color_image_callback(self, msg): 
```
디버깅용 이미지 콜백함수.  
어차피 이미지 구독은 퍼블리셔 쪽에서 하고 화면 출력도 하기 때문에 구독자에선 굳이 화면을 띄울 필요는 없지만,  
위의 판단 기준과 점선을 그리는 기준이 옳게 작동 중인지 확인하기 위해 넣은 콜백함수.  
점선과 근사 직선을 그리고, 판단 결과를 글씨로 화면에 띄운다.

### 4) ball_detect  
요약: 농구공을 탐지하고 중심 좌표를 퍼블리시하는 코드. line_publisher와 거의 같다.  

line 코드는 이름을 publisher, subscriber로 했으면서  
왜 ball은 detect, recieve로 이름을 지었을까?  
나도 모르겠다.  
역할은 다르지 않다.

```python
class BasketballDetectorNode(Node):
    def __init__(self):
        super().__init__('basketball_detector')
        ...
    def parameter_callback(self, params):
        ...
        return SetParametersResult(successful=True)
    
    def click(self, event, x, y, _, __): 
        ...
    def image_callback(self, color_msg: Image, depth_msg: Image):
        ...
```
이 구조는 이제 익숙해졌을 것이다.

line_publisher와의 차이점을 조금 설명하자면,  
1. BallTracker의 부재  
    농구공은 객체간의 구분이 필요 없는 단일 객체이기 때문에, 딕셔너리로 묶어서 관리할 필요가 없다.  
    때문에 추적기 class를 만들지 않았다.  
    대신 추적기 역할은 있긴 해야하므로, 그 기능은 농구공 탐지 조건문인
```python
for cnt in contours:
```
이 뒤에 구현되어있다  
자세한 내용은 주석 참고.

2. 거리 센서 활용  
    거리 센서 활용했다.

말고는 거의 비슷한 구조일 것이다.

realsense에서 이미지 노드 받아오고 > 전처리 > 추론 > 후처리 > 퍼블리시  
의 구조는 웬만하면 같을 것이기 때문에

### 5) ball_recieve
사실상 line_subscriber와 거의 동일하다.  
메세지 타입의 특징 때문에 유효성 검사 부분 코드가 다를 뿐

### \# 그외 다수

### 1) hoop_detect

### 2) hurdle_detect

### 3) ball_and_hoop

### 4) line_tracker



## 1-2. launch
여러 노드(주로 실행파일)들을 한 번에 실행하고, 설정까지 할 수 있는 런치 파일들을 모아놓는 폴더.

사실 여기에 만들어둔 launch 폴더는 realsense2_camera 패키지의 launch 폴더 예시라서 당장은 필요없다.  
그냥 설명만 붙여놓겠음.
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

***


## 1-3. package.xml

## 1-4. setup.cfg

## 1-5. setup.py



# robot_msgs
ros2 토픽, 서비스 등에 사용되는 메세지들을 정의하는 패키지

자세한 건 핑크랩,,,

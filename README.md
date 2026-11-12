# 🐟 bungeoppang — 붕어빵 굽는 로봇팔

두산 협동로봇이 손님을 맞이하고, 음성으로 주문을 받아, 반죽 붓기부터 속재료 넣기·틀 닫기·뒤집기·꺼내기까지 붕어빵 한 판을 처음부터 끝까지 굽는 ROS 2 패키지입니다.
RViz2 + MoveIt 2 시뮬레이션(두산 가상 에뮬레이터)에서 전체 공정을 검증하고, 실물 **Doosan E0509** 적용을 준비하고 있습니다.

| 항목 | 내용 |
|---|---|
| 로봇 | Doosan E0509 (실물) / M1013 (가상 시뮬레이션) |
| 그리퍼 | ROBOTIS RH-P12-RN |
| ROS | ROS 2 Jazzy, `ament_python` |
| 모션 | MoveIt 2 + pymoveit2 (IK, 충돌 검사, 직선 경로) |
| 주문 인식 | faster-whisper (한국어 음성 인식) |
| 손님 감지 | OpenCV + MediaPipe 얼굴 검출 |
| 시각화 | RViz2 (MarkerArray, 상황판 Image) |

---

## 동작 흐름

```
손님 도착 감지 → 90도 인사 → 주문 듣기 (종류·개수)
   ├─ 알아들음  → 3초 끄덕끄덕
   └─ 못 알아들음 → 도리도리 후 다시 질문 (최대 3회)
→ 틀 개수만큼 나눠서 굽기 → 90도 인사 → 다음 손님
```

한 판을 굽는 순서는 다음과 같습니다.

1. 반죽통 잡고 들어올리기 → 1차 반죽 붓기 → 제자리
2. 속재료 넣기 (팥 = 빨간 통, 슈크림 = 노란 통, 각 틀마다 아래 → 위로 짜기)
3. 2차 반죽 붓기 → 반죽통 제자리
4. 철판 뚜껑 한 번에 닫기 (180°)
5. 윗면 굽기 → 회전 손잡이로 철판 180° 뒤집기 → 뒷면 굽기 → 되돌리기
6. 뚜껑 열기 → 붕어빵 꺼내서 쟁반에 놓기

### 이동 원칙

물건을 들고 있을 때는 **위로 → 수평 → 아래로**, 직선 이동만 합니다. 직선 경로는 1cm / 2° 간격으로 가장 가까운 관절 해를 이어 붙여 계산하고, 관절이 튀는 해는 거부해서 불필요한 360° 회전이 생기지 않게 했습니다. 예외적인 회전은 반죽통 기울이기와 소스통 입구를 틀 쪽으로 뒤집는 동작뿐입니다. 팔(link_3~5)에는 안전 패딩을 두고 경로를 일정 각도 간격으로 충돌 검사합니다.

---

## 노드 구성

```
 customer ──/bungeoppang/customer──▶ ┌──────────────┐ ──/bungeoppang/listen──▶ voice_order
 (웹캠/키보드)                        │  pour_demo   │ ◀──/bungeoppang/order─── (마이크/키보드)
                                     │ (로봇 공정)   │
                                     └──────┬───────┘
                                            │ /bungeoppang/step
                                            ▼
                       hud ──/bungeoppang/hud (Image)──▶ RViz2 상황판
```

| 실행 이름 | 파일 | 역할 |
|---|---|---|
| `pour_demo` | `pour_demo.py` | 메인 공정 노드. `layout.py`의 배치를 MoveIt 장면에 올리고 가게 운영 루프 실행 |
| `pour_e0509` | `pour_e0509.py` | E0509 + 작업대 120×50cm 전용 단독 버전 (설정 내장) |
| `my_node` | `my_node.py` | 초기 버전: M1013, 틀 3개, 음성 주문 전체 공정 |
| `voice_order` | `voice_order_node.py` | 주문 요청을 받으면 한 번 듣고 `{"counts": {"팥": n, "슈크림": m}}` 발행 |
| `customer` | `customer_node.py` | 얼굴이 일정 시간 보이면 `arrived`, 사라지면 `left` 발행 |
| `hud` | `hud_node.py` | 손님·로봇 질문·들은 말·인식 결과·현재 단계를 한글 이미지로 그려 발행 |
| `reach_check` | `reach_check.py` | 로봇을 움직이지 않고 공정의 주요 목표 자세마다 IK·충돌 여부를 표로 출력 |
| `gripper_marker` | `gripper_marker.py` | `link_6` 기준 그리퍼 모양 마커 (RViz2용) |

공용 모듈은 다음 두 개입니다.

- `layout.py` — 로봇 설정, 작업 파라미터, 장면 배치(base_link 기준, m). 모델마다 다른 값은 `PROFILES`에만 적고 나머지는 거기서 계산합니다.
- `geom.py` — 벡터·쿼터니언·보간 등 ROS와 무관한 계산 함수.

### 음성 주문 인식

`voice_order`는 faster-whisper(`small`, CPU int8)로 5초간 듣고, 메뉴 힌트를 프롬프트로 넣어 인식률을 높입니다. 인식 결과는 규칙 기반 파서로 개수를 뽑아냅니다. "팥붕 두 개 슈붕 하나", "세 개 다 팥", "두 개씩", "슈크림은 빼고" 같은 표현과 "수붕", "팟봉"처럼 자주 틀리는 발음도 처리하며, 한 번에 최대 6개까지 받습니다.

---

## 설치

```bash
# 의존성 (예시)
sudo apt install ros-jazzy-moveit
pip install pymoveit2 faster-whisper sounddevice opencv-python mediapipe pillow --break-system-packages

# 두산 ROS 2 패키지(dsr_bringup2 등)가 워크스페이스에 있어야 합니다
cd ~/ros2_ws/src
git clone https://github.com/SujinJi/bungeoppang.git
cd ~/ros2_ws
colcon build --packages-select bungeoppang
source install/setup.bash
```

---

## 실행

### 한 번에 실행 (`start_all.sh`)

정리 → 빌드 후 MoveIt + RViz2, 로봇, 음성 주문, 손님 감지, 상황판 창을 각각 엽니다.

```bash
~/ros2_ws/start_all.sh                  # pour_demo, 테스트 모드 (키보드로 손님·주문)
~/ros2_ws/start_all.sh my_node          # my_node, 테스트 모드
~/ros2_ws/start_all.sh pour_demo voice  # 마이크로 주문, 손님 도착은 키보드
~/ros2_ws/start_all.sh pour_demo real   # 실제 카메라 · 마이크
```

'로봇' 창에 "손님을 기다리는 중"이 뜨면 '손님 감지' 창에서 Enter를 누르면 시작합니다.

### 따로 실행

```bash
# 1) 두산 가상 로봇 + MoveIt + RViz2
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m1013 host:=127.0.0.1

# 2) 로봇 공정 (move_group·컨트롤러 준비를 기다린 뒤 실행)
~/ros2_ws/run_demo.sh pour_demo

# 3) 주변 노드
ros2 run bungeoppang voice_order --ros-args -p mode:=text   # 키보드 주문
ros2 run bungeoppang customer --ros-args -p mode:=key       # Enter = 손님 도착
ros2 run bungeoppang hud                                    # RViz2 Image 패널에서 /bungeoppang/hud
```

### 환경 변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `ROBOT_MODEL` | `e0509` | `layout.py` 배치 선택. `m1013` = 전체 공정 완주가 확인된 가상 배치, `e0509` = 실물 배치 |
| `BUNGEO_VEL` | 모델별 | 속도 배율 덮어쓰기 (예: 가상에서 `BUNGEO_VEL=0.2`) |

실물 E0509는 처음에 `vel 0.05`로 느리게 시작합니다.

### 도달 검사

배치를 바꾼 뒤에는 로봇을 움직이기 전에 먼저 확인합니다.

```bash
ROBOT_MODEL=e0509 ros2 run bungeoppang reach_check
```

실패한 자세가 있으면 `layout.py`의 `PROFILES['e0509']` 좌표를 조정합니다. 목표 지점만 검사하므로, 지점 사이 직선 경로의 관절 튐은 `pour_demo` 실행 중에 따로 확인됩니다.

### 보조 스크립트

| 스크립트 | 설명 |
|---|---|
| `run_demo.sh [노드]` | move_group·`dsr_controller2` 준비 대기, 중복 노드 확인, MoveIt 파라미터 적용, 빌드 후 실행 |
| `run_my.sh` | 같은 준비 과정 후 `my_node` 실행 |
| `clean.sh` | ROS·MoveIt·두산 프로세스와 도커 컨테이너 정리, ROS 데몬 재시작 |
| `backup.sh "메모"` | `~/bungeoppang_backups/`에 날짜·메모 이름으로 패키지 백업 |

---

## 작업 공간 배치 (E0509)

- 작업대 120 × 50cm, 로봇은 바닥판(27 × 26cm) 위에 설치
- 로봇 정면(+x) = 작업대 긴 방향, +y = 짧은 방향
- 틀 2개를 위아래 철판으로 묶어 한 번에 닫고 뒤집음
- 소스통 2개(팥·슈크림)는 틀 뒤쪽 받침대 위에 나란히, 쟁반은 로봇 왼쪽 아래
- 로봇이 작업대 오른쪽으로 너무 가면 멈추므로 안전 거리를 유지

모든 좌표와 공정 파라미터(붓는 높이·기울기, 굽는 시간, 그리퍼 폭 등)는 `layout.py` 한 곳에서 관리합니다. 테스트 중에는 `TIME_SCALE = 0.1`로 대기 시간을 줄이고, 실제 굽기에서는 `1.0`으로 바꿉니다.

---

## 디렉터리 구조

```
bungeoppang/
├── bungeoppang/
│   ├── pour_demo.py            # 메인 공정 (모델별 배치)
│   ├── pour_e0509.py           # E0509 단독 버전
│   ├── my_node.py              # 초기 M1013 버전
│   ├── layout.py               # 로봇·파라미터·배치
│   ├── geom.py                 # 계산 도우미
│   ├── voice_order_node.py     # 음성 주문
│   ├── customer_node.py        # 손님 감지
│   ├── hud_node.py             # 상황판
│   ├── reach_check.py          # 도달 검사
│   └── gripper_marker.py       # 그리퍼 마커
├── urdf/
├── resource/  test/
├── package.xml  setup.py  setup.cfg
└── start_all.sh  run_demo.sh  run_my.sh  clean.sh  backup.sh
```

---

## 진행 상황

- [x] M1013 가상 환경에서 음성 주문 → 굽기 → 접시까지 전체 공정 완주
- [x] 손님 감지, 음성 주문, 상황판 노드
- [x] E0509 실물 배치 프로필과 도달 검사 도구
- [ ] 실물 E0509 + RH-P12-RN 그리퍼에서 동작 확인 및 파라미터 튜닝
- [ ] 실물 그리퍼 제어 연동

## License

Apache-2.0

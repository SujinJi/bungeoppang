"""로봇 설정 · 작업 파라미터 · 장면 배치 (base_link 기준, m)

로봇 모델은 환경변수 ROBOT_MODEL 로 고름 (기본 e0509 = 실물).
  ROBOT_MODEL=m1013  → 예전 M1013 가상 시뮬레이션 배치 (전체 공정 완주 확인된 값)
  ROBOT_MODEL=e0509  → 실물 E0509 배치 (도달 900mm). 좌표는 1차 추정치 → reach_check 로 맞출 것
속도는 BUNGEO_VEL 로 덮어쓸 수 있음 (예: 가상에서 BUNGEO_VEL=0.2).

모델마다 다른 값은 PROFILES 에만 적고, 나머지는 아래에서 그 값으로 계산함.
"""
import math
import os

# ---------------- 모델별 배치 ----------------
PROFILES = {
    "m1013": {
        "TZ": 0.0,                         # 작업대 윗면 높이 (로봇 바닥 기준)
        "TABLE_SIZE": [1.60, 1.60, 0.04],
        "TABLE_XY": [0.55, 0.0],
        "MACHINE_X": 0.90,                 # 기계 중심 (MACHINE_Y 없으면 y = 0)
        "MACHINE_L": 0.50,                 # 기계 앞뒤 길이: 틀 + 뒤로 180도 열린 뚜껑
        "MOLD_X0": 0.70,                   # 틀 앞쪽 끝 (뒤쪽 = MOLD_X0 + MOLD_L)
        "KETTLE_XY": [0.50, 0.35],
        "SAUCES": {"팥": [0.45, -0.50], "슈크림": [0.66, -0.70]},   # 팔이 다른 소스통 위로 지나가지 않는 자리
        "SAUCE_GRASP_YAW": 60.0, "SAUCE_YAW": 60.0,
        "PLATE_XY": [0.30, 0.60],          # 왼쪽 (주전자 뒤쪽)
        "PLATE_SIZE": [0.30, 0.30, 0.01],
        "PLATE_SPOTS": [(dx, dy, 0) for dx in (0.075, -0.075) for dy in (0.07, 0.0, -0.07)],
        "CUSTOMER_DIR": 90.0,
        "SAFE_Z": 0.55,
        "TRANSIT_ZS": (0.55, 0.45, 0.38),
        "P": {"vel": 0.2},
    },
    "e0509": {
        # 작업대 가로(동서) 120 x 세로(남북) 50cm, 로봇 바닥판 27x26cm (두께 2.5cm)
        # 로봇 중심 = 작업대 서쪽 끝에서 25.8cm, 북쪽 끝에서 15.5cm, 남쪽 끝에서 34.5cm (pour_e0509.py 실측)
        # 로봇 정면(+x) = 동쪽, +y = 북쪽
        "OBJECT_SHIFT": [0.20, 0.0],       # 로봇·작업대 빼고 물체 전부 x +20cm (아래 좌표는 옮기기 전 값)
                                           # 접시는 안 옮김: 옮기면 로봇 바닥판 옆(x 0.03)이라 안 닿음
        "TZ": -0.025,
        "TABLE_SIZE": [1.20, 0.50, 0.04],
        "TABLE_XY": [0.342, -0.095],
        "ROBOT_PLATE": [0.27, 0.26],       # 로봇 바닥판 (장애물로 장면에 넣음)
        "MOLD_COUNT": 2,                   # 틀 2개 (한 판에 2개씩), 위아래 철판으로 한 번에 닫고 뒤집음
        "MOLD_SIZE": [0.133, 0.08, 0.04],  # 틀 2/3 크기 (길이, 폭, 깊이) - 원래 20 x 12 x 6cm
        "MOLD_PITCH": 0.093,               # 틀 간격 2/3 (원래 14cm)
        "FISH_SIZE": [0.093, 0.04, 0.017], # 붕어빵도 2/3
        "MACHINE_W": 0.22,
        "MACHINE_L": 0.37,                 # x 0.23 ~ 0.60: 틀 + 뒤로 180도 열린 뚜껑
        "MACHINE_X": 0.415,
        "MACHINE_Y": -0.10,                # 틀 y -0.054 / -0.147
        "MOLD_X0": 0.28,                   # 틀이 짧아져서 3cm 뒤로 (붓는 자리가 로봇에 너무 붙지 않게)
        "KETTLE_XY": [0.03, -0.26],        # 주둥이(+x)가 틀에 닿지 않는 자리
        "SAUCES": {"팥": [0.66, -0.06], "슈크림": [0.66, -0.26]},  # 틀 뒤 (열린 뚜껑 너머) 나란히 20cm 간격 (북쪽에 붙이면 팔이 회전 손잡이에 걸림)
        "SAUCE_TURN_X": 0.54,              # 소스통 든 손 방향 돌리는 곳 (이동 후 0.74: 옆 소스통과 12cm, 0.78 이상이면 오른쪽 틀에서 안 닿음)
        "SAUCE_GRASP_YAW": 0.0,            # 로봇 쪽(서쪽)에서 잡기 (북/남쪽에서 잡으면 0.77m라 안 닿음), 나란히라 서로 안 지나감
        "SAUCE_YAW": 60.0,                 # 짤 때 손 방향. -90은 joint_4 한계, +90은 오른쪽 틀에서 joint_5 한계(±135)
        "SAUCE_GRASP_ROLL": 180.0,         # 그리퍼를 자기 축으로 180도 돌려 잡기 (평행 그리퍼라 잡는 모양 같음)
                                           # → joint_6이 ±180 경계에서 떨어짐
        "SAUCE_STAND_H": 0.13,             # 소스통 받침대 13cm: 잡는 높이 0.20m 이상이어야 팔이 열린 뚜껑에 안 닿음
        "SAUCE_CARRY_Z": 0.30,             # 소스통 들고 이동하는 높이 (틀 위에서 손 방향 돌리고 뒤집기 높이로 올라감)
        "PLATE_XY": [-0.17, -0.235],       # 로봇 왼쪽 아래 쟁반
        "PLATE_SIZE": [0.17, 0.22, 0.01],
        "PLATE_SPOTS": [(0.0, dy, lv) for lv in (0, 1, 2) for dy in (0.0, -0.065)],   # 2칸 x 3층 (북쪽 칸은 로봇 바닥판에 붙어서 안 닿음)
        "PLATE_VIA": [0.12, -0.28],        # 틀 → 쟁반 갈 때 거치는 곳 (로봇 바로 위를 지나지 않게)
        "CUSTOMER_DIR": -90.0,             # 손님 방향: 작업대 아래쪽(-y)
        "SAFE_Z": 0.45,
        "TRANSIT_ZS": (0.45, 0.38, 0.32),  # 0.55m는 E0509가 못 올라감, 먼 틀 위는 0.32m
        "P": {"vel": 0.05,                 # 실물 처음엔 0.05 (CLAUDE.md 실물 안전)
              "pour_clear": 0.03,          # 주둥이 끝이 틀 위 3cm. 높으면 붓기에서 joint_5 한계(±135도) 넘음
              "pour_tilt": 25.0,           # 30도는 joint_5 한계 → 25도 (joint_5 약 129도)
              "fish_lift": 0.20,           # 틀이 멀어서 30cm 위(0.39m)는 안 닿음 → 높이 약 0.30m
              "pour2_lift": 0.10,          # E0509는 30cm까지 못 올라감
              "flip_clear": 0.06,          # 소스통 뒤집을 때 노즐 끝이 틀 위 6.5cm를 지나감 (높으면 안 닿음)
              "bake_front_s": 0.0,         # 닫자마자 뒤집기
              "lid_pitch": -30.0},         # 열린 손잡이가 멀어서 30도 기울여 잡기 (닫힘 쪽은 가까워서 0도)
    },
}

MODEL = os.environ.get("ROBOT_MODEL", "e0509").strip().lower()
if MODEL not in PROFILES:
    raise SystemExit(f"ROBOT_MODEL={MODEL!r} 배치 없음 → {', '.join(PROFILES)} 중 하나")
_L = dict(PROFILES[MODEL])

# 물체 전체 옮기기 (로봇·작업대는 그대로): OBJECT_SHIFT = [dx, dy]
_dx, _dy = _L.get("OBJECT_SHIFT", (0.0, 0.0))
if _dx or _dy:
    for _k in ("MACHINE_X", "MOLD_X0", "SAUCE_TURN_X"):
        if _k in _L:
            _L[_k] = round(_L[_k] + _dx, 6)
    _L["MACHINE_Y"] = round(_L.get("MACHINE_Y", 0.0) + _dy, 6)
    for _k in ("KETTLE_XY",):
        if _L.get(_k):
            _L[_k] = [round(_L[_k][0] + _dx, 6), round(_L[_k][1] + _dy, 6)]
    _L["SAUCES"] = {k: [round(v[0] + _dx, 6), round(v[1] + _dy, 6)] for k, v in _L["SAUCES"].items()}

# ---------------- 로봇 설정 (두산 공통) ----------------
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
BASE_LINK = "base_link"
EE_LINK = "link_6"
GROUP = "manipulator"
HOME = [0.0, 0.0, math.pi / 2, 0.0, math.pi / 2, 0.0]
WRAP_JOINTS = (0, 3, 5)
YAW_OFFSET = 180.0

TIME_SCALE = 0.1           # 대기 시간 배율: 테스트 0.1, 실제 1.0
CUSTOMER_DIR = _L.get("CUSTOMER_DIR", 90.0)   # 손님이 서 있는 방향 (로봇 기준, 도). 0 = 정면, -90 = 오른쪽
ORDER_WAIT = 25.0          # 주문 답을 기다리는 시간 (초)
ORDER_TRIES = 3            # 주문을 물어보는 최대 횟수

# ---------------- 작업 파라미터 (실물 튜닝 대상) ----------------
P = {
    "vel": 0.2,
    "joint_vmax": 2.0,
    "lin_speed": 0.10,
    "ang_speed": 20.0,
    "path_step": 0.01,
    "path_deg": 2.0,
    "max_jump_deg": 25.0,
    "ik_timeout": 0.05,      # IK 한 번에 쓰는 시간 (초). E0509 MoveIt 기본 0.015로는 먼 자세를 못 찾음
    "safety_pad": 0.02,      # 안전 거리: 팔(link_3~5)을 이만큼 두껍게 보고 충돌 검사 (m)
    "check_deg": 3.0,        # 경로를 몇 도 간격으로 충돌 검사할지
    "tcp": 0.15,
    "approach": 0.08,
    # 인사 / 반응
    "bow_shoulder": 45.0,
    "bow_elbow": 45.0,
    "bow_hold": 1.5,
    "yes_s": 3.0,
    "yes_deg": 20.0,
    "no_deg": 15.0,
    # 주전자
    "kettle_lift": 0.20,
    "pour_point": 2 / 3,
    "pour_clear": 0.10,
    "pour_tilt": 30.0,
    "pour1_hold": 3.0,
    "pour2_hold": 1.0,
    "pour_hop": 0.08,
    "pour2_lift": 0.30,
    # 소스통
    "sauce_lift": 0.30,
    "flip_clear": 0.10,
    "nozzle_gap": 0.005,
    "dispense_front": 0.04,
    "dispense_back": 0.05,
    "dispense_speed": 0.03,
    "dispense_hop": 0.08,
    # 굽기
    "lid_step": 5.0,         # 뚜껑(윗 철판)을 몇 도씩 돌릴지 (열림 180도 ↔ 닫힘)
    "lid_pitch": 0.0,        # 열린 뚜껑 손잡이 잡는 그리퍼 기울기 (음수 = 손끝이 로봇 바깥쪽), 닫힐수록 0으로
    "flip_step": 10.0,       # 틀을 몇 도씩 돌릴지
    "bake_s": 180.0,
    "bake_front_s": 90.0,    # 뚜껑 닫고 뒤집기 전에 굽기 (0이면 닫자마자 뒤집기)
    "bake_back_s": 90.0,     # 180도 뒤집은 채로 굽기
    "open_wait": 10.0,
    "fish_lift": 0.30,       # 붕어빵 들고 접시로 갈 때 틀 바닥에서 올라가는 높이
}
P.update(_L.get("P", {}))
if os.environ.get("BUNGEO_VEL"):
    P["vel"] = float(os.environ["BUNGEO_VEL"])

GRIP_OPEN = 0.10
GRIP_KETTLE = 0.02
GRIP_SAUCE = 0.07
GRIP_SQUEEZE = 0.055
GRIP_LID = 0.06
GRIP_FISH = 0.06

# ---------------- 장면 배치 (base_link 기준, m) ----------------
TZ = _L["TZ"]
GAP = 0.002
WALL = 0.01
TABLE_SIZE = list(_L["TABLE_SIZE"])
TABLE_POS = [_L["TABLE_XY"][0], _L["TABLE_XY"][1], TZ - TABLE_SIZE[2] / 2 - GAP]

MACHINE_SIZE = [_L.get("MACHINE_L", 0.30), _L.get("MACHINE_W", 0.50), 0.02]   # 틀이 앞뒤로 넘어갈 공간을 위해 얇게
MACHINE_POS = [_L["MACHINE_X"], _L.get("MACHINE_Y", 0.0), TZ + 0.01]
MOLD_COUNT = _L.get("MOLD_COUNT", 3)          # 붕어빵 틀 개수 = 한 판에 굽는 개수
MOLD_NAMES = {2: ["왼쪽", "오른쪽"], 3: ["왼쪽", "중간", "오른쪽"]}[MOLD_COUNT]
MOLD_PITCH = _L.get("MOLD_PITCH", 0.14)
MOLD_YS = [MACHINE_POS[1] + MOLD_PITCH * ((MOLD_COUNT - 1) / 2 - k) for k in range(MOLD_COUNT)]   # +y(왼쪽)부터
MOLD_L, MOLD_W, MOLD_D = _L.get("MOLD_SIZE", [0.20, 0.12, 0.06])   # 틀 길이(x), 폭(y), 깊이(z)
MOLD_X0 = _L["MOLD_X0"]
MOLD_X1 = round(MOLD_X0 + MOLD_L, 6)          # 반올림: 부동소수 오차 없이 계산
MOLD_XC = (MOLD_X0 + MOLD_X1) / 2
MOLD_BASE = TZ + 0.10
MOLD_TOP = MOLD_BASE + MOLD_D
POUR_X = MOLD_X0 + MOLD_L * P["pour_point"]

# 철판: 틀 전부를 아래 철판 1장, 뚜껑 전부를 위 철판 1장에 붙임 → 손잡이 하나로 한 번에 닫고 엶
STEEL_T = 0.005
STEEL_Y0 = min(MOLD_YS) - MOLD_W / 2 - 0.01   # 철판 y 범위
STEEL_Y1 = max(MOLD_YS) + MOLD_W / 2 + 0.01
STEEL_W = STEEL_Y1 - STEEL_Y0
STEEL_YC = (STEEL_Y0 + STEEL_Y1) / 2

# 뚜껑 = 위쪽 틀 반쪽 (아래 틀과 같은 두께). 경첩은 틀 뒤쪽 윗모서리
# 뚜껑 각도 phi: -90 = 뒤로 180도 열려 아래 틀 옆에 누움 (처음 상태), +90 = 닫힘
# 뚜껑 좌표 (lx, lz): phi=0(세운 상태)에서 lx = 두께 방향(+x), lz = 길이 방향(+z)
HINGE_X, HINGE_Z = MOLD_X1 + 0.005, MOLD_TOP
LID_LEN, LID_T = MOLD_L, MOLD_D
LID_OPEN, LID_CLOSED = -90.0, 90.0
LID_HANDLE = [LID_T / 2, LID_LEN + STEEL_T + 0.015]   # 위 철판 끝에 튀어나온 손잡이 (뚜껑 좌표)
LID_HANDLE_SIZE = [0.02, 0.08, 0.03]                  # (lx, y, lz)

# 틀 뒤집기: 닫힌 철판(아래 철판+틀+뚜껑+위 철판)을 회전 손잡이로 가로(y) 축 기준 180도 회전
AXIS_Z = MOLD_TOP                            # 회전축 높이 (닫힌 틀의 가운데 = 아래·위 틀 맞닿는 면)
AXIS_X = (MOLD_X0 + HINGE_X) / 2             # 회전축 앞뒤 위치 (닫힌 틀의 가운데)
CRANK = [AXIS_X, STEEL_Y1 + 0.045, AXIS_Z]  # 기계 왼쪽(+y) 옆면 회전 손잡이
CRANK_SIZE = [0.03, 0.03, 0.03]
CRANK_YAW = -90.0                                       # 그리퍼가 기계 쪽(-y)을 향하게
FLIP_ROLL0 = 90.0
GRIP_FLIP = 0.03

KETTLE_R, KETTLE_H = 0.07, 0.16
SPOUT_LEN = 0.14
KX, KY = _L["KETTLE_XY"]
KZ = TZ + GAP + KETTLE_H / 2
KETTLE_GRIP = [KX, KY, TZ + GAP + KETTLE_H + 0.05]
SPOUT_TIP = [KETTLE_R + SPOUT_LEN, (KZ + 0.04) - KETTLE_GRIP[2]]

# 소스통 2개: 팥 = 빨간 통 (로봇 가까운 쪽), 슈크림 = 노란 통 (먼 쪽)
SAUCES = {k: list(v) for k, v in _L["SAUCES"].items()}
SAUCE_IDS = {"팥": "sauce_red", "슈크림": "sauce_cream"}
FILL_COLORS = {"팥": (0.55, 0.12, 0.08), "슈크림": (1.0, 0.85, 0.3)}
FILL_LABELS = {"팥": "RED BEAN", "슈크림": "CUSTARD"}
SAUCE_R, SAUCE_H, NOZZLE_H = 0.035, 0.20, 0.04
NOZZLE_TIP = SAUCE_H / 2 + NOZZLE_H
SAUCE_STAND_H = _L.get("SAUCE_STAND_H", 0.0)   # 소스통 받침대 높이 (0이면 받침대 없음)
SAUCE_Z0 = TZ + GAP + SAUCE_STAND_H             # 소스통 바닥 높이
SIDE = -90.0
SAUCE_YAW = _L.get("SAUCE_YAW", 60.0)               # 짤 때 손 방향
_g = _L.get("SAUCE_GRASP_YAW", SAUCE_YAW)
SAUCE_GRASP_YAW = dict(_g) if isinstance(_g, dict) else {k: _g for k in SAUCES}   # 통마다 잡을 때 손 방향
SAUCE_TURN_X = _L.get("SAUCE_TURN_X", MOLD_X1 + 0.15)   # 소스통 든 채 손 방향 돌리는 x 위치
SAUCE_GRASP_ROLL = _L.get("SAUCE_GRASP_ROLL", 0.0)   # 소스통 잡는 그리퍼 축 회전 (뒤집으면 여기서 ±180)
SAUCE_CARRY_Z = _L.get("SAUCE_CARRY_Z", SAUCE_Z0 + SAUCE_H / 2 + P["sauce_lift"])   # 소스통 들고 이동 높이

# 접시(쟁반): 붕어빵 자리 (dx, dy, 층)
PLATE_XY, PLATE_SIZE = list(_L["PLATE_XY"]), list(_L["PLATE_SIZE"])
FISH_SIZE = list(_L.get("FISH_SIZE", [0.14, 0.06, 0.025]))
PLATE_SPOTS = list(_L["PLATE_SPOTS"])
PLATE_VIA = _L.get("PLATE_VIA")               # 틀 → 쟁반 사이 거치는 곳 (없으면 바로)
MAX_ORDER = len(PLATE_SPOTS)

# 안전 이동
SAFE_Z = _L["SAFE_Z"]                          # 바로 가면 위험할 때 올라갈 기본 높이 (m)
TRANSIT_ZS = tuple(_L["TRANSIT_ZS"])          # 높은 것부터 시도 (멀리 있는 목표는 너무 높으면 팔이 안 닿음)
PAD_LINKS = ["link_3", "link_4", "link_5"]   # 안전 거리를 적용할 팔 부분

ROBOT_PLATE = _L.get("ROBOT_PLATE")           # 로봇 바닥판 [가로, 세로] (없으면 장면에 안 넣음)

KETTLE_PARTS = ["kettle", "kettle_handle", "kettle_post1", "kettle_post2", "kettle_spout"]


def sauce_grip(kind):
    x, y = SAUCES[kind]
    return [x, y, SAUCE_Z0 + SAUCE_H / 2]


def sauce_parts(kind):
    return [SAUCE_IDS[kind], SAUCE_IDS[kind] + "_nozzle"]

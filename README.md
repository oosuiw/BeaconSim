# BeaconSim — 세이프티 비콘 RGL 경량 심

## 바로 실행 (AppImage)

[릴리스](https://github.com/oosuiw/BeaconSim/releases)에서 `BeaconSim-<버전>-x86_64.AppImage`를 받아:

```bash
chmod +x BeaconSim-*-x86_64.AppImage
./BeaconSim-*-x86_64.AppImage                                   # GUI (처음 실행 시 비콘 CAD STL 선택)
./BeaconSim-*-x86_64.AppImage scenarios [YAML|폴더] [--ros]      # 회귀 테스트 (기본: 동봉 시나리오)
./BeaconSim-*-x86_64.AppImage export-map --seed 7 ~/maps/s7      # Autoware 맵 세트
./BeaconSim-*-x86_64.AppImage build-robot --stl CAD.stl          # 로봇 모델 생성 (~/.config/beaconsim)
```
- PC에 ROS2 Humble이 있으면 자동으로 토픽 발행·`/cmd_vel` 수신 (`BEACONSIM_NO_ROS=1` 로 끔)
- Tele-15: `~/.config/beaconsim/tele.csv`가 있으면 실제 패턴, 없으면 자체 로제트 근사 패턴
- 빌드: `packaging/build_appimage.sh`

## 설치 (소스)

요구 사항: Ubuntu 22.04, NVIDIA RTX GPU + 드라이버 515 이상(RGL), OpenGL 3.3, (선택) ROS2 Humble.

```bash
git clone https://github.com/oosuiw/BeaconSim.git && cd BeaconSim
python3 -m venv --system-site-packages .venv          # ROS2 rclpy를 쓰려면 system-site-packages 필요
.venv/bin/pip install -U pip && .venv/bin/pip install -r requirements.txt
```

### 직접 내려받을 제3자 자료 (저장소에 포함하지 않음)

| 넣을 위치 | 내용 | 받는 곳 |
|---|---|---|
| `third_party/rgl/libRobotecGPULidar.so` | RGL v0.21.0 **core** 빌드 (ROS 확장 없는 것) | [RobotecGPULidar 릴리스](https://github.com/RobotecAI/RobotecGPULidar/releases) → `RGL-core-linux-x64.zip` |
| `third_party/ref/tele.csv` | Livox Tele-15 비반복 스캔 패턴 | [livox_laser_simulation](https://github.com/lvfengchi/livox_laser_simulation) → `scan_mode/tele.csv` |

헤더(`include/rgl/api/core.h`)는 참고용이라 필요 없습니다. Pandar40P 채널 표는 AWSIM(`LaserArrayLibrary.cs`) 값을 코드에 옮겨 두었습니다.
카메라(Arducam B0495) STEP·데이터시트는 제조사 사이트에서 받으세요.

### 로봇 모델 (팀 내부 자료)

비콘 CAD(`절곡 프레임 설계 v2.stl`)는 공개 저장소에 없습니다. 팀 내부에서 받아 저장소 루트에 둔 뒤:

```bash
python3 tools/build_robot.py        # → robot/meshes/*, config/robot.yaml (센서 장착 포즈)
```

## 실행

```bash
./run_app.sh                 # 통합 시뮬레이터 GUI (맵·시나리오·교통·환경·센서 시각화, ROS2 있으면 토픽 발행)
./run_sim.sh                 # 헤드리스 ROS2 심 (+ 카메라 노드). rviz:=true 로 RViz도 가능
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0}}"   # 상위 제어기 대신 시험

# GUI 없이 맵 세트만 만들기
.venv/bin/python -m beacon_sim.export --seed 7 --lanes 2 --length 3000 ~/BeaconSim_maps/s7
```

### 통합 시뮬레이터 (run_app.sh)
시작 화면에서 모드를 고르고 조건(맵 · 시나리오 · 교통 · 환경, 시험이면 시험 설정)을 정한 뒤 시작한다.
프리셋과 "YAML 불러오기"(회귀 시나리오나 시험 결과의 scenario.yaml)로 같은 조건을 재현할 수 있다. 사이드바 🏠 → 시작 화면.

| 모드 | 용도 | 실행 중 조건 변경 |
|---|---|---|
| 개발 | 센서 · TF · 상위 제어기 연동 디버깅 (기본: 교통 끔, 평가 없음) | 바꾸면 "적용"으로 바로 재시작 |
| 시험 | 조건 1세트로 평가 → PASS/FAIL 리포트 (기본: 교통 켬) | **잠금**. 바꾼 값은 다음 시험용, "이 조건으로 재시작" 시 현재 결과 저장 후 새 시험 |
| 회귀 | `scenarios/` YAML 일괄 실행 (tools/run_scenarios.py), 선택적 ROS 연동 | - |

- 시험 모드 설정: 이름, 시험 시간(또는 제한 없음), 판정 기준(예시값, 수정 가능), 센서 고장 주입 구간, ROS 평가 토픽(측위 · 차선검출)
- 보기 설정(시점, 센서 표시)은 언제든 변경 가능. 배속 변경과 키보드 조작(수동 개입)은 리포트에 기록
- 결과: `results/gui/<시각>_<이름>/` (AppImage는 `~/BeaconSim_results/gui/`) — `report.md`, `result.json`, `timeseries.csv`,
  `scenario.yaml`(랜덤 항목을 실제 값으로 고정 → 회귀 테스트에 그대로 사용). 결과 창의 "회귀 시나리오로 저장" → `scenarios/`
- 명령행: `--mode dev|test` 로 시작 화면 건너뛰기, `--scenario X.yaml` 로 조건 불러오기
- 탭: 맵 / 시나리오 / 교통 / 환경 / 보기 / 저장. 상단: 일시정지, 배속(0.25~4x), 시나리오 재시작(R), 맵 재생성(G), 시드 랜덤 체크
- 비콘 조작: ROS `/cmd_vel`(상위 제어기) 우선, 없으면 방향키(↑전진 ↓후진 ←→회전, 속도는 시나리오 탭)
- 보기: Pandar40P(세기/정답 라벨 색), Tele-15, ARS408 클러스터·객체 상자, GNSS 위치·오차원·궤적(정답 궤적 흰색), 카메라 창,
  SCC/운전자 상태 점(하늘 SCC 작동, 보라 SCC가 비콘 추종 + 연결선, 노랑 운전자 인지, 빨강 추돌)
- 3D 뷰: 좌드래그 회전, 우드래그 이동, 휠 확대, F 비콘 따라가기, 1 비콘 2 전체 3 터널
- 설정 저장 위치: `~/.config/beaconsim/`

### 기능 검증 모드 / 시드 랜덤
- 교통 탭 "접근 차량 켜기" 끄기 (또는 sim.yaml `traffic.enabled: false`) → 차량 없이 비콘·사고 현장만
- 시나리오·맵 시드: GUI 체크박스, sim.yaml 은 `seed: -1` 이면 실행마다 랜덤 (결정값은 로그와 `/sim/scenario`)

### SCC (적응형 순항) 차량
- 탑재 비율·차간 시간(1.0~2.5 s)·최대 감속도(2.5~5 m/s²) 랜덤. 상용 SCC처럼 **정지한 비콘은 무시**하고,
  비콘이 교통 흐름 방향(사고 현장 쪽)으로 움직이면 앞차로 인식해 추종·감속 (한 번 잡으면 비콘이 멈춰도 추적 유지)
- 운전자가 반응 중이면 사람 급제동(8 m/s²), SCC만 작동 중이면 SCC 최대 감속도로 제한

### 운전자 인지 · 비콘 충돌
- 인지 거리 = 기본(60~250 m) × 시간대(낮 1.0, 황혼 0.85, 밤 0.9 / 경광등 끄면 0.35) × 날씨(비·눈 −35%×강도, 안개 −20%×강도),
  상한 = 가시거리×0.8. **시야 가림**: 운전자 눈 → 비콘 RGL 레이 (앞 대형차·곡선부 지형·터널 벽에 가리면 인지 못 함)
- 비콘 외형(CAD)과 사고 차량·주행 차량·가드레일·중앙분리대·터널 벽 충돌 → 이동 막힘 + `beacon_contact` 이벤트
- 차량이 비콘 추돌 → 비콘이 차량 방향으로 밀려나고(최대 25 m) **파손** (이후 명령 무시), `beacon_damaged` 이벤트

### 자동 평가 / 회귀 테스트 (tools/run_scenarios.py, scenarios/)
```bash
.venv/bin/python tools/run_scenarios.py scenarios/                 # 전체, ROS 없이 실시간 4~6배속
.venv/bin/python tools/run_scenarios.py scenarios/S09_scc_backing.yaml
source /opt/ros/humble/setup.bash && .venv/bin/python tools/run_scenarios.py scenarios/ --ros   # 상위 제어기·측위·차선검출 연동
```
- 시나리오: S01 개활지 기준선 · S02 차선 마모 · S03 터널 입구 · S04 장대 터널 · S05 터널 출구 · S06 대형차 혼잡(가림) ·
  S07 센서 고장(Pandar/레이더/GNSS 드롭) · S08 밤+비 전체 · S09 SCC 후진 인식 · S10 기능 검증(차량 없음)
- YAML: `map/scenario/traffic/environment` 덮어쓰기, `faults`, `beacon_script`(상위 제어기 자리표시 — `--ros` 시 /cmd_vel 우선),
  `topics`(측위 Odometry/PoseStamped, 차선검출 PointCloud2), `criteria`(합격 기준 — **값은 예시**, 실차 요구사항에서 역산해 수정)
- 지표: 추돌·비콘 추돌·최소 TTC·비콘 차로 최소 간격·통과 속도/횡간격·인지 거리·차로변경/감속/SCC 인식·비콘 앞 정지 차량 /
  비콘 파손·접촉·이동거리 / GNSS FIX 비율·NO FIX 시간·재수렴 시간 / (토픽 연결 시) 측위 횡·종·헤딩 오차, 차선검출 정밀도·재현율
- 결과: `results/<시각>/report.md`, `report.json`, `<시나리오>/timeseries.csv`. FAIL 하나라도 있으면 종료 코드 1 (CI용)

### 환경 (environment.py)
- 시간대: 낮/황혼/밤 (밤: 가로등·차량 전조등/후미등, 영상 노이즈 증가), 날씨: 맑음/비/안개/눈 + 강도
- 카메라: 조도·대기 산란(가시거리)·빗줄기/눈송이·노이즈 / LiDAR: 대기 감쇠, 비·눈 근거리 허위점, 젖은 노면·차선 반사율 저하 /
  레이더: SNR 저하 / GNSS: 영향 없음. 모두 실측으로 보정할 간이 모델

### GNSS (gnss.py, ZED-F9P)
- RTK FIX(σ 1.5 cm) / FLOAT(30 cm) / SINGLE(1.5 m) / NO FIX, 시간 상관 오차
- 터널 안 NO FIX, 갱문 주변 SINGLE, 진출 후 SINGLE → FLOAT → (10~60 s 재수렴) → FIX. `/sensing/gnss/fix` (NavSatFix)

### 차선 상태
- 구간별 양호/희미함(반사율·색 대비 저하)/끊김(도색 벗겨짐)/희소(점선 일부, 실선 토막) — 비율·유형 가중치 조절
- Lanelet2 맵에는 선이 그대로 남고 센서(카메라/LiDAR)에만 반영

### 2차 사고 시나리오 (beacon_sim/traffic.py, config/sim.yaml 의 scenario:/traffic:)
- 사고 현장: 비콘 쪽 방향(하행/상행 랜덤)의 랜덤 차로에 사고 차량 1~3대 (최소 한 차로는 통행 가능, 나머지는 길어깨)
- 비콘: 사고 차량 후미에 **교통 흐름 반대 방향**으로 배치만 한다. **이동은 상위 제어기가 `/cmd_vel`로** (시뮬레이터는 움직이지 않음)
- 접근 차량: IDM 추종, 비콘 인지 거리(랜덤) 안에 들어오면 반응 — 차로변경 / 감속 / 무반응(추돌). 비율·교통량·속도·인지 거리 조절
- 토픽: `/sim/traffic`(PoseArray, 슬롯 순서 고정), `/sim/events`(JSON: brake / lane_change / crash), `/sim/scenario`(JSON, latched)
- GUI에서는 비콘이 배치 위치에 고정된 상태로 교통 반응을 미리 본다

### 도로 생성 규칙 (beacon_sim/roadgen.py)
- 평면선형: 직선 → 클로소이드 → 원곡선 → 클로소이드 반복. 최소반경 460 m(100 km/h) / 710 m(120 km/h)
- 종단경사 최대 3 %, 이동평균으로 종단곡선 근사. 편경사: 최소반경에서 6 %, 반경이 클수록 감소(최소 2 %), 직선부 바깥쪽 2 %
- 횡단: 중앙분리대(0.6 m) + 안쪽 길어깨 1.0 m + 차로 3.6 m × N + 바깥 길어깨 3.0 m
- 차선: 중앙분리대 쪽 황색 실선, 차로 사이 백색 점선(10 m/10 m), 길어깨 경계 백색 실선. 마모 구간은 렌더/센서에만 반영(맵에는 선 유지)
- 터널: 방향별 단독 터널 + 갱문 + 산지, 터널 내부 조명 감쇠(카메라)
- 좌표: MGRS 100 km 격자 로컬 좌표 (기존 Autoware 맵의 local_x/local_y와 동일 규칙으로 검증). 격자 원점 + (3 km, 3 km)에서 시작해 float32 정밀도 문제 회피
- 같은 (시드, 파라미터, GENERATOR_VERSION) → 같은 맵

## 구성

| 파일 | 역할 |
|---|---|
| `tools/build_robot.py` | 설계 STL → `base_link` 기준 부품 메시(`robot/meshes`) + CAD 측정 센서 포즈(`config/robot.yaml`) |
| `config/robot.yaml` | 센서 장착 포즈. URDF(TF)와 심 센서가 **같은 값**을 쓴다 |
| `config/sim.yaml` | 토픽, 주기, 센서 파라미터 |
| `beacon_sim/engine.py` | 시뮬레이션 엔진 (ROS 비의존): 도로·시나리오·교통·환경·RGL 센서·GNSS·비콘 기구학 |
| `beacon_sim/ros_bridge.py` | 엔진 ↔ ROS2 토픽 (GUI·헤드리스 공용) |
| `beacon_sim/sim_core.py` | 헤드리스 ROS2 노드 (엔진 + 브리지) |
| `beacon_sim/gnss.py`, `environment.py` | ZED-F9P 모델, 시간대/날씨 모델 |
| `beacon_sim/camera_node.py` | Arducam B0495 (EGL OpenGL 렌더 + 왜곡 remap) |
| `beacon_sim/roadgen.py` | 시드 기반 도로 모델 (선형/종단/편경사/터널/갠트리/마모) |
| `beacon_sim/scene.py` | 도로 모델 → 씬 메시 |
| `beacon_sim/traffic.py` | 사고 현장·비콘 배치, 접근 차량 교통 시뮬레이션 |
| `beacon_sim/export.py` | Lanelet2 OSM, map_projector_info, PCD 저장 + 검증 |
| `beacon_sim/app.py` | 통합 시뮬레이터 GUI (Dear ImGui + moderngl + GLFW) |
| `beacon_sim/render.py`, `imgui_mgl.py` | 공용 OpenGL 씬 렌더러, ImGui moderngl 백엔드(한글 동적 폰트) |
| `tools/smoke_test.py` | ROS 없이 센서 1프레임 통계 |
| `tools/capture_check.py` | 실행 중인 심에서 점군/이미지를 map 좌표로 그림 저장 |

## 토픽

| 토픽 | 형식 | 프레임 |
|---|---|---|
| `/sensing/lidar/pandar/pointcloud_raw_ex` | PointCloud2, nebula PointXYZIRCAEDT (32B) | `pandar_40p` |
| `/ground_truth/lidar/pandar/labels` | PointCloud2 x,y,z,intensity,label,instance | `pandar_40p` |
| `/sensing/lidar/livox/points` | PointCloud2, livox_ros_driver2 형식 (26B) | `livox_tele15` |
| `/sensing/radar/ars408/{clusters,scan,objects,object_markers}` | PointCloud2 / RadarScan / RadarTracks / Marker | `ars408` |
| `/sensing/camera/arducam/{image_raw,camera_info}` | Image rgb8 + CameraInfo | `camera_optical_link` |
| `/sensing/gnss/fix` | NavSatFix (status: GBAS=RTK FIX, SBAS=FLOAT, FIX=SINGLE) | `gnss_link` |
| `/ground_truth/odom`, TF `map→base_link` | 정답 포즈 | |

라벨 id: 1 road, 2 lane_white, 3 lane_yellow, 4 guardrail, 5 barrier, 6 terrain, 7 tunnel, 8 vehicle, 9 robot, 10 sign, 11 pole, 12 accident

### 수신 쪽 영상·점군이 빠질 때 (DDS 버퍼)
카메라 영상(약 7 MB)·Pandar 점군(약 1.8 MB)은 UDP 조각으로 나뉘어 전달된다. 받는 쪽 수신 버퍼(`net.core.rmem_max`/`rmem_default`)가
작으면 넘쳐서 메시지가 통째로 버려진다 (시뮬레이터는 10 Hz 로 내보내는데 받는 쪽은 2~3 Hz). 한 번만 실행:
```bash
sudo tools/setup_ros_network.sh      # /etc/sysctl.d/99-beaconsim-ros.conf (재부팅 후에도 유지)
```
수신 주기 확인은 `ros2 topic hz` 대신 `python tools/topic_rates.py 10` (메시지를 풀지 않아 측정 도구가 병목이 되지 않음).
버퍼가 작으면 앱 시작 시 이벤트 창에 경고가 나온다. 성능 로그: `BEACONSIM_PERF=1 ./run_godot.sh` (fps·RTF·센서 ms·카메라 왕복).

## 의존성

- RGL v0.21.0 core 바이너리: `third_party/rgl/` (ROS 확장 없는 빌드. full 빌드의 ROS 라이브러리는 rclpy와 충돌 가능)
- `.venv` (system-site-packages + moderngl, imgui-bundle, glfw). Open3D 오프스크린 렌더러가 이 환경에서 크래시해서 moderngl 사용
- UI 폰트: `assets/fonts/` (Pretendard PUA 제거본 + Font Awesome 6, 둘 다 OFL). 없으면 나눔고딕·은돋움으로 대체
- 스캔 패턴/채널 표: `third_party/ref/` (livox_laser_simulation tele.csv, AWSIM LaserArrayLibrary)

## 라이선스
Copyright (c) 2026 BeaconSim authors. All rights reserved. (LICENSE 참고)
코드는 참고용으로 공개되며, 사용·수정·배포에는 저작권자의 서면 허락이 필요합니다.
함께 쓰는 제3자 구성요소(RGL, Pretendard, Font Awesome, Python 패키지)는 각자의 라이선스를 따릅니다 (packaging/THIRD_PARTY.md).

## Godot 카메라 렌더러 (시제품)
기존 카메라(moderngl)보다 고품질인 카메라 영상을 Godot 4 (Forward+)로 그린다. 엔진·교통·LiDAR/레이더는 그대로 쓴다.
- 준비: [Godot 4.x Linux](https://godotengine.org/download/linux/)를 받아 `third_party/godot/` 에 압축 해제 (또는 `BEACONSIM_GODOT=<실행 파일>`)
- 비교: `.venv/bin/python tools/godot_camera.py --times 8 --set environment.time_of_day=night --set environment.weather=rain`
  → `results/godot/<시각>/compare_<t>.png` (위: 기존, 아래: Godot, 같은 렌즈 왜곡 적용)
- 실시간: `--live 30` (Godot 창에 카메라 주기로 송출)
- 구성: `beacon_sim/godot_link.py` (glb·json 내보내기, UDP 상태 송신), `godot/` (Godot 프로젝트)
- GUI: 보기 탭 → 카메라 렌더러 "Godot 고품질" (또는 `--camera godot`). 카메라 창과 ROS 이미지 토픽이 Godot 영상으로 바뀐다.
  Godot 은 1x1 투명 창으로 따로 실행되고(최소화하면 렌더링이 멈춤), 영상은 /dev/shm 으로 받는다. 맵·시나리오를 다시 만들면 약 5초 동안 기본 렌더러.
- AppImage: `WITH_GODOT=1 packaging/build_appimage.sh` 로 Godot 실행 파일 포함. 없으면 `~/.config/beaconsim/godot/` 에 두면 된다.
- 아직: 헤드리스(sim_core + camera_node) 경로는 기본 렌더러, 차량 창문·번호판 등 세부 모델 없음

## Godot 프런트엔드 (개발 중, B안)
화면·UI 를 Godot 4 로 옮기는 중. 파이썬 백엔드(`beacon_sim/server.py`)가 엔진·센서·ROS 를 맡고, Godot 창(`godot/client.gd`)과 TCP 로 연결한다.
- 실행: `./run_godot.sh` (옵션: `--no-ros`, `--size 3440x1400`, `--set env.weather=rain`)
- 1단계(완료): 3D 월드 고품질 렌더, 마우스 시점(좌 회전·우 이동·휠 확대, F 따라가기, 1/2/3 시점), 방향키 비콘 주행,
  Space/R/G, Pandar·Tele·레이더(클러스터·앞면 판)·GNSS·SCC 표시, 재생 바·상황판·이벤트·카메라 패널,
  ROS 카메라 토픽(Godot 렌더 → 렌즈 왜곡 → 발행)
- 2단계(완료): 사이드바 탭(맵·시나리오·교통·환경·보기·저장)과 이벤트 피드(전체/차량), 도움말(F1), 상황판 범례,
  레이더 감지 범위 면, 차량 이름표(추돌·SCC 추종 속도), Pandar 정답 라벨 색, Lanelet 선, 지형·터널·차량·비콘 표시 켜고 끄기, UI 배율.
  탭 항목은 파이썬(`beacon_sim/ui_schema.py`)이 정의하고 Godot(`godot/sidebar.gd`)이 그린다 → 항목 추가는 파이썬만 고치면 된다.
  생성 조건·저장 위치는 기존 앱과 같은 `~/.config/beaconsim/app.json`, 화면 표시·방향키 설정은 `godot_ui.json`
- UI 자체 시험: `./run_godot.sh --no-ros --selftest results/ui_selftest` (가상 마우스·키 입력으로 탭·체크박스·슬라이더·단축키를 눌러 보고 화면 저장)
- 3단계(완료): 시작 화면(개발/시험/회귀 카드, 프리셋·YAML 불러오기·기본값, 조건 탭, 요약, 최근 결과), 시험 모드
  (사이드바 진행 카드·수동 개입 수·시험 종료, 조건 잠금 배너 → "이 조건으로 재시작"/되돌리기, 시험 설정 탭: 판정 기준·고장 주입·평가 토픽),
  결과 창(기준 표·결과 폴더·회귀 시나리오로 저장·다시 시험), 회귀 실행(시나리오 선택·ROS 연동·진행·report.md), 첫 실행 STL 선택.
  화면·시험 로직 중 UI 와 무관한 부분은 `beacon_sim/modes.py` 로 빼서 기존 앱과 같이 쓴다.
  새 장면을 화면이 다 불러온 뒤에 엔진이 출발하고(시험은 보이는 순간 t=0), 시험 시간에는 엔진이 정확히 멈춘다.
- 명령행: `./run_godot.sh --mode test --scenario scenarios/S01_open_clear.yaml` (시작 화면 없이 바로), `--robot-stl <CAD.stl>`
- 다음: 4단계 AppImage 전환
- 그동안 기존 GUI(`run_app.sh`)는 그대로 사용 가능

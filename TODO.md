# 나중에 고칠 것 (실기 값 확인 필요)

임시 값으로 동작 중인 항목. 실기 사양·실측을 받으면 아래 위치를 고친다.

## 주행 (`config/sim.yaml` → `drive:`)
- [ ] `gear_ratio` — 모터 → 바퀴 감속비 (지금 10, 임시)
- [ ] `counts_per_motor_rev` — 모터 1회전당 엔코더 카운트 (지금 4096, 임시)
- [ ] `motor_max_rpm` — 모터 최대 회전수 (지금 3000 rpm, 임시)
- [ ] `tau`, `accel_max` — 바퀴 속도 응답·가속 한계 (실측 스텝 응답으로 맞출 것)
- [ ] `track_eff_factor`, `odom_track_factor` — 스키드 스티어 유효 윤거 배율 (제자리 회전 실측으로 맞출 것, 지금 1.35)
- [ ] `slip_std` — 바퀴 미끄럼 잡음

## IMU (`config/sim.yaml` → `imu:`)
- [ ] 기종 확인 후 `gyro_noise`, `gyro_bias_walk`, `accel_noise`, `accel_bias_walk` 를 데이터시트 값으로
- [ ] `xyz` — base_link 기준 장착 위치 (지금 [0, 0, 0.25], 임시), 장착 방향이 다르면 rpy 추가
- [ ] `topic`, `frame_id` — 실기 드라이버와 같은 이름으로

## 센서타워 (`config/robot.yaml` → `sensor_tower:`)
- [ ] `deploy_time` — 접힘 → 펴짐 시간 (지금 4.0 s, 가정)

## GNSS (`config/sim.yaml` → `gnss:`)
- [ ] `antenna_xyz` — 안테나 위치 실측

## 회귀 시나리오 (`scenarios/*.yaml` → `criteria:`)
- [ ] 합격 기준은 예시 값 — 실차 요구사항에서 역산해 수정 (S01, S09 는 지금 기준으로 FAIL)

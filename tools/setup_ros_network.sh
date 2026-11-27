#!/usr/bin/env bash
# ROS 2 (CycloneDDS) 큰 메시지 수신 버퍼 설정 — 카메라 영상(약 7 MB)·LiDAR 점군이 받는 쪽에서 버려지지 않게.
#   sudo tools/setup_ros_network.sh        (한 번만, 재부팅 후에도 유지)
# 원인: UDP 수신 버퍼(net.core.rmem_max/default)가 작으면 조각난 큰 메시지가 넘쳐서 통째로 버려진다.
#       /etc/sysctl.d/60-zed-buffers.conf 처럼 1 MB 로 낮추는 설정이 뒤에 적용되면 앞의 큰 설정이 덮인다.
# 이 파일(99-…)은 가장 나중에 적용되어 그 값을 덮는다. ZED 스트리밍은 1 MB 이상이면 되므로 영향 없음.
set -e
if [ "$(id -u)" -ne 0 ]; then
  echo "관리자 권한 필요: sudo $0"; exit 1
fi
CONF=/etc/sysctl.d/99-beaconsim-ros.conf
cat > "$CONF" <<'EOF'
# BeaconSim: ROS 2 / CycloneDDS 큰 메시지(카메라 영상·점군) 수신 버퍼
net.core.rmem_max=2147483647
net.core.rmem_default=16777216
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sysctl --system > /dev/null
echo "적용: $CONF"
sysctl net.core.rmem_max net.core.rmem_default net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh
echo "이미 실행 중인 ROS 노드는 다시 시작해야 새 버퍼를 씁니다."

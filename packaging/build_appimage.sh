#!/usr/bin/env bash
# BeaconSim AppImage 빌드.  사용: packaging/build_appimage.sh [버전]
#  - 파이썬 3.10 독립 런타임(python-build-standalone) + 의존 패키지 + 코드/리소스 + RGL core(Apache-2.0)
#  - 비콘 CAD 파생물(로봇 메시·포즈)과 tele.csv(라이선스 없음)는 넣지 않는다 → 첫 실행 시 STL 선택, Tele-15는 자체 패턴
#  - 실행 PC에 ROS2 Humble이 있으면 AppRun이 setup.sh를 불러 rclpy 사용 (없으면 단독 실행)
set -euo pipefail
export PYTHONNOUSERSITE=1          # 빌드 PC의 ~/.local 패키지를 보지 않게
unset PYTHONPATH PYTHONHOME        # ROS 등 외부 경로 차단
cd "$(dirname "$(readlink -f "$0")")/.."
VER="${1:-$(python3 -c 'import sys; sys.path.insert(0,"."); from beacon_sim.roadgen import GENERATOR_VERSION as v; print(v)')}"
PY_URL="https://github.com/astral-sh/python-build-standalone/releases/download/20261009/cpython-3.10.22%2B20261009-x86_64-unknown-linux-gnu-install_only.tar.gz"
RGL_URL="https://github.com/RobotecAI/RobotecGPULidar/releases/download/v0.21.0/RGL-core-linux-x64.zip"
TOOL_URL="https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage"
B=build/appimage
APPDIR=$B/BeaconSim.AppDir
DL=$B/downloads
rm -rf "$APPDIR"; mkdir -p "$APPDIR/usr" "$DL"

fetch() { [ -f "$2" ] || curl -fL --retry 3 -o "$2" "$1"; }
fetch "$PY_URL" "$DL/python.tar.gz"
fetch "$RGL_URL" "$DL/rgl.zip"
fetch "$TOOL_URL" "$DL/appimagetool"; chmod +x "$DL/appimagetool"

echo "== 파이썬 런타임"
tar -xzf "$DL/python.tar.gz" -C "$APPDIR/usr"          # → usr/python
PY="$APPDIR/usr/python/bin/python3"
"$PY" -m pip install -q --no-cache-dir -U pip
grep -vE "^\s*(#|matplotlib)" requirements.txt > "$B/req.txt"; "$PY" -m pip install -q --no-cache-dir --ignore-installed -r "$B/req.txt"
"$PY" -c "import numpy, scipy, cv2, yaml, pyproj, moderngl, glfw, imgui_bundle, fontTools; print('패키지 확인 OK', numpy.__version__)"
# 크기 줄이기: 테스트/캐시 제거
find "$APPDIR/usr/python" -name '__pycache__' -type d -prune -exec rm -rf {} +   # tests 폴더는 남김 (numpy.testing이 런타임에 참조)
rm -rf "$APPDIR/usr/python/lib/python3.10/idlelib" "$APPDIR/usr/python/lib/python3.10/tkinter" \
       "$APPDIR/usr/python/lib/python3.10/ensurepip"

echo "== 앱 리소스"
R="$APPDIR/usr/share/beaconsim"
mkdir -p "$R/third_party/rgl" "$R/LICENSES"
cp -r beacon_sim config assets scenarios tools README.md "$R/"
rm -f "$R/config/robot.yaml"; find "$R" -name '__pycache__' -prune -exec rm -rf {} +
unzip -q -o "$DL/rgl.zip" -d "$DL/rgl"
cp "$(find "$DL/rgl" -name libRobotecGPULidar.so | head -1)" "$R/third_party/rgl/"
fetch https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/v0.21.0/LICENSE "$DL/RGL-LICENSE"
cp "$DL/RGL-LICENSE" "$R/LICENSES/RobotecGPULidar-Apache-2.0.txt"
cp assets/fonts/OFL-Pretendard.txt "$R/LICENSES/"
cp packaging/THIRD_PARTY.md "$R/LICENSES/"

echo "== AppRun / desktop / 아이콘"
cp packaging/AppRun "$APPDIR/AppRun"; chmod +x "$APPDIR/AppRun"
cp packaging/beaconsim.desktop "$APPDIR/"
"$PY" packaging/make_icon.py "$APPDIR/beaconsim.png"
mkdir -p "$APPDIR/usr/share/icons/hicolor/256x256/apps"
cp "$APPDIR/beaconsim.png" "$APPDIR/usr/share/icons/hicolor/256x256/apps/"

echo "== AppImage"
OUT="$B/BeaconSim-${VER}-x86_64.AppImage"
ARCH=x86_64 "$DL/appimagetool" --appimage-extract-and-run --no-appstream "$APPDIR" "$OUT" >/dev/null
ls -la "$OUT"

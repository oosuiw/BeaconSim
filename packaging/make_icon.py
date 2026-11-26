"""앱 아이콘 (256x256 PNG): 어두운 둥근 사각형 + 경고 삼각형 + 비콘 전파."""
import sys

import cv2
import numpy as np

S = 256
img = np.zeros((S, S, 4), np.uint8)
mask = np.zeros((S, S), np.uint8)
cv2.rectangle(mask, (24, 0), (S - 25, S - 1), 255, -1)
cv2.rectangle(mask, (0, 24), (S - 1, S - 25), 255, -1)
for c in [(24, 24), (S - 25, 24), (24, S - 25), (S - 25, S - 25)]:
    cv2.circle(mask, c, 24, 255, -1)
img[mask > 0] = (36, 26, 20, 255)                                   # BGRA 짙은 남색
tri = np.array([[128, 58], [210, 196], [46, 196]], np.int32)
cv2.fillPoly(img, [tri], (40, 170, 255, 255), cv2.LINE_AA)           # 주황 경고 삼각형
cv2.fillPoly(img, [np.array([[128, 92], [182, 182], [74, 182]], np.int32)], (36, 26, 20, 255), cv2.LINE_AA)
cv2.rectangle(img, (120, 120), (136, 160), (40, 170, 255, 255), -1, cv2.LINE_AA)
for r, a in [(70, 255), (92, 170)]:                                  # 전파 호
    cv2.ellipse(img, (128, 70), (r // 2, r // 2), 0, 210, 330, (255, 163, 77, a), 6, cv2.LINE_AA)
cv2.imwrite(sys.argv[1], img)

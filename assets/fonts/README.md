# UI 폰트

| 파일 | 출처 | 라이선스 |
|---|---|---|
| `Pretendard-{Medium,SemiBold,Bold}.otf` | [Pretendard](https://github.com/orioncactus/pretendard) 원본 그대로 | SIL OFL 1.1 (`OFL-Pretendard.txt`, 예약 이름 'Pretendard') |
| `Font_Awesome_6_Free-Solid-900.otf` | [Font Awesome 6 Free](https://fontawesome.com) (imgui_bundle 동봉본) | 폰트: SIL OFL 1.1 |

Pretendard에는 PUA(U+E000–F8FF) 영역에 자체 글리프가 있어 Font Awesome 아이콘을 가립니다.
앱이 처음 실행될 때 `~/.cache/beaconsim/fonts/` 에 PUA만 뺀 사본을 만들어 씁니다 (fontTools 필요, 배포하지 않음).

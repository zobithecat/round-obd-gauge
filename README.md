# Round OBD Gauge 🦞

1.28인치 원형 디스플레이 보드(**ESP32-2424S012**, GC9A01 240×240 IPS + CST816D 터치)용
**멀티펑션 OBD-II BLE 게이지**입니다. ELM327 계열 BLE 어댑터에 연결해 실시간 차량 데이터를
대담하고 부드러운 원형 게이지로 보여줍니다.

[LovyanGFX](https://github.com/lovyan03/LovyanGFX) 기반. 싱글코어 ESP32-C3, PSRAM 없음 —
모든 렌더링이 8-bit 풀스크린 스프라이트 하나로 돌아갑니다.

![board: ESP32-2424S012](https://img.shields.io/badge/board-ESP32--2424S012-red) ![mcu: ESP32--C3](https://img.shields.io/badge/mcu-ESP32--C3-blue)

## 기능

- **게이지 5종** — RPM · 속도 · 냉각수온 · 부스트 · 점화시기 — 스와이프로 전환
- **차오르는 아크 스타일** + 게이지별 색 그라데이션(바늘 없음), 안티앨리어싱 캡
- **0점 중앙 게이지** — 부스트·점화시기는 0을 12시에 두고 양방향으로 채움
- **터치(CST816D)** — 스와이프 = 게이지 전환, 단일 탭 = 피크홀드, 롱프레스 = 피크 리셋
- **0–100 km/h 가속 타이머(제로백)** — 속도 게이지에서 더블탭, 현대 N모드 풍 빨강 테마
- **연료 등급 판정(저급/일반/고급)** — 절대 진각이 아니라 *부하 시 knock-retard*로 추론
- **과열 / 레드라인 / 오버부스트 경고** — 임계 초과 시 빨간 링 점멸
- **부트 세레머니** — OpenClaw 로고가 점점 커지며 등장(AA 줌)
- **데모 페이스** — BLE 미연결 60초 지속 시 합성 데이터로 게이지가 살아 움직임
- **6번째 스와이프 = 설정 메뉴** — 로깅 ON/OFF 토글(NVS에 영구 저장)
- **NVS 진단 로그** — 로깅을 켜면 부팅 후 첫 60초의 연결 진단(스캔/GATT/응답)을 NVS에 저장하고,
  다음 부팅 때 시리얼로 덤프. **차에서 켜고 주행 → 책상에서 USB만 꽂으면 지난 로그 확인** (노트북 불필요)
- **견고한 BLE 라이프사이클** — 빠른 연결, ~2.5초 끊김 감지, ~5초 자동 재연결,
  끊기면 값 0으로 리셋 + 칼만 필터 재초기화로 즉시 회복
- ~44 FPS 풀프레임 리드로우, CAN 안전 폴링(≤10 req/s, 요청-응답 페이싱)

## 구성

```
round_obd_gauge/   게이지 펌웨어 (PlatformIO, env esp32c3)
mock_icar_pro/     벤치 테스트용 가짜 "IOS-Vlink" OBD2 BLE 어댑터 (ESP32-S3)
```

## 빌드 & 플래시

```bash
# 게이지 (ESP32-C3 원형 디스플레이)
pio run -d round_obd_gauge -e esp32c3 -t upload

# Mock 어댑터 — 보드에 맞는 env 선택:
pio run -d mock_icar_pro -e esp32s3       -t upload   # 네이티브 USB S3 (4MB)
pio run -d mock_icar_pro -e esp32s3_uart  -t upload   # CH340 UART S3 (16MB)
```

게이지는 `IOS-Vlink`로 광고하는 BLE 어댑터에 자동 연결됩니다
(`round_obd_gauge.ino`의 `OBD_BLE_NAME`으로 변경).

## Mock 어댑터 시리얼 명령 (115200)

`1`/`2`/`3`/`4` = 공회전 / 주행 / 스포츠 / 과열 · `q`/`w`/`e` = 저급/일반/고급 연료 ·
`s` = 상태 · `h` = 도움말

## 하드웨어 메모

- 디스플레이 GC9A01: SCLK=6 MOSI=7 DC=2 CS=10, 백라이트=GPIO3, `invert=true`
- 터치 CST816D: SDA=4 SCL=5 INT=0 RST=1 (주소 0x15)
- 16-bit 풀스크린 스프라이트는 C3에서 할당 실패(SRAM 단편화) — 캔버스는 8-bit 사용
- 실제 어댑터 대응: 파서가 응답 어디서든 `41XX` 패턴 검색(`SEARCHING`/헤더 접두사 대응),
  notify 또는 indicate 구독 모두 지원

---
🤖 [Claude Code](https://claude.com/claude-code)로 제작

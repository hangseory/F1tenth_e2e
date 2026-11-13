# Cartographer ver. 매뉴얼

맵핑 → 저장 → 정제 → 라인 → 설정 → 주행. 명령은 그대로 복사해 붙이세요.

| | 어디서 | 무엇 |
|---|---|---|
| 터미널 A | 젯슨 SSH | 맵핑 / 주행 (켜 둔 채로) |
| 터미널 B | 젯슨 SSH | 저장 / 정제 / 라인 / 설정 |
| 노트북 | 로컬 | RViz, 제어판 |

```bash
ssh jetson@<젯슨IP>          # 비밀번호 jetsonjetson
ip -4 -br addr | grep -v docker   # 젯슨 IP = wlP1p1s0 줄 (enP8p1s0 는 라이다 전용)
```

> ⚠ 스택을 켠 뒤 와이파이를 바꾸면 통신이 조용히 끊깁니다 → `./race.sh stop` 후 재시작.

---

## 제어판 (선택 — 터미널을 하나로)

```bash
cd ~/yax && ./panel.sh --server                          # ① 젯슨. 세 줄 찍고 멈춘 듯 보이면 정상
ssh -N -L 8765:127.0.0.1:8765 jetson@<젯슨IP> &         # ② 노트북 터널. 아무 출력 없음이 정상
cd ~/yax && ./panel.sh                                    # ③ 노트북 창 (PyQt5 만 필요)
```

---

## 1. 맵핑 — 터미널 A

차를 **출발시킬 자리·방향에 놓고** 시작합니다 (그 자리가 맵 원점).
```bash
cd ~/yax && ./race.sh mapping-carto-joy
```

## 2. 저장 — 터미널 B (A 를 끄기 전에!)

```bash
cd ~/yax && ./race.sh savemap-carto 1109
```
```

## 3. 맵 정제 — 터미널 B

```bash
cd ~/yax && ./race.sh clean 1109
```
자동이 엉뚱한 영역을 고르면 폴리곤으로 (`1` 바깥 → `2` 안쪽 → `Enter` → `s` 저장):
```bash
cd ~/yax && DISPLAY=:1 python3 src/race_stack/maps/clean_map.py --map 1109
```


## 4. 라인 — 터미널 B

```bash
cd ~/yax && ./race.sh line 1109_clean                 # 자동 (빠르지만 벽에 더 붙음)
cd ~/yax && ./race.sh waypoints-kappa 1109_clean      # 수동, GUI 필요
cd ~/yax && ./race.sh mintime 1109_clean --sketch --car car1 --out 1109_clean_wp_mt1   # 최소시간 (docs/mintime_라인_매뉴얼.md)
```

- **`★★ ... 이 라인은 못 돈다` 가 뜨면** 그 라인은 못 씁니다 → 아래 좁은 트랙 옵션으로.
- 좁은 트랙: `cd ~/yax && ./race.sh line 1109_clean --safety 0.10 --wallclear 0.06 --apex-extra 0 --step 0.08`
- **재생성하면 같은 이름 라인을 덮어씁니다** → `--out <새이름>` 을 주세요.
- 조향 여유가 부족하면 `--kappa-margin 0.65` (여유 30% 이상 권장).

```bash
cd ~/yax && ./race.sh wp          # wp all = 전부, wp c11 = 이름 검색
```

라인 손보기 (`c` 로 닫힌 루프 확인, `Enter` 저장 → `_edit.csv` 로 따로 저장):
```bash
cd ~/yax && DISPLAY=:1 python3 -m race_stack.waypoint_editor src/race_stack/maps/1109_clean.yaml --out 1109_clean_wp_kappa.csv
```


## 5. 설정 — 터미널 B

```bash
cd ~/yax && ./tune.sh localization.mode=cartographer && ./tune.sh map_name=1109_clean && ./tune.sh waypoints=1109_clean_map_wp
```
`.pbstream` 은 `map_name` 에서 `_clean` 을 떼어 자동으로 찾습니다. (트랙 설정은 커밋하지 않습니다.)

초기 위치 — 클릭 1 = 위치, 클릭 2 = 방향, `w` 저장. 차도 그 자리·방향에 놓습니다:
```bash
cd ~/yax && ./race.sh start_pos --map 1109_clean --wp-file 1109_clean_map_wp --gui
```

## 6. 확인 → 주행 — 터미널 A

정지 상태로 띄워 RViz 에서 **스캔(빨간 점)이 맵 벽과 겹치는지** 확인:
```bash
cd ~/yax && ./race.sh stop && ./race.sh box
```

어긋났으면 RViz `2D Pose Estimate` 로 찍고 (차 정지 상태에서만 받음), 파일에 저장:
```bash
cd ~/yax && ./race.sh savepose
```
RViz 로 찍은 위치는 저장되지 않습니다 → `savepose` / `start_pos` / `set_init_pose.py --write` 후 **스택 재시작**.

저속 시험 → 속도 올리기:
```bash
cd ~/yax && ./tune.sh cap=0.5 && ./race.sh q1      # cap 은 다음 실행에도 남는다
cd ~/yax && ./tune.sh cap=0 && ./tune.sh q1=0.8     # 괜찮으면 천장 풀고 올리기
cd ~/yax && ./race.sh stop && ./race.sh race        # 콘 회피까지
```

- 출발 직후 느린 건 정상 (`[출발제한]`, 휠 스핀으로 인한 위치 오차 방지).
- 랩 내내 느리면 `./tune.sh cap` 이 0 인지 확인.
- **Cartographer 는 길을 잃으면 스스로 복구하지 못합니다.** 불안하면 `./tune.sh localization.mode=amcl` 로 되돌리기.

범퍼 후진 설정:
```bash
cd ~/yax && ./tune.sh bump_reverse.trigger_gap          # 현재 값 보기 → 0.03
cd ~/yax && ./tune.sh bump_reverse.trigger_gap=0.05     # 바꾸기
cd ~/yax && ./tune.sh bump_reverse.enabled=false        # 기능 끄기
```

## 7. 마무리

```bash
cd ~/yax && ./race.sh stop
```

---

## 명령 한 장 요약

```bash
# A — 맵핑
cd ~/yax && ./race.sh mapping-carto-joy
# B — 저장 → 정제 → 라인 → 설정
cd ~/yax && ./race.sh savemap-carto 1109
cd ~/yax && ./race.sh clean 1109
cd ~/yax && ./race.sh line 1109_clean
cd ~/yax && ./tune.sh localization.mode=cartographer && ./tune.sh map_name=1109_clean && ./tune.sh waypoints=1109_clean_map_wp
cd ~/yax && ./race.sh start_pos --map 1109_clean --wp-file 1109_clean_map_wp --gui
# A — 확인 후 주행
cd ~/yax && ./race.sh stop && ./race.sh box
cd ~/yax && ./tune.sh cap=0.5 && ./race.sh q1
# 노트북
cd ~/yax && ./rviz.sh car1
```

## 잘 안 될 때

| 증상 | 조치 |
|---|---|
| `Already exists ... Choose a new map name` | 새 이름 쓰기 |
| `.pbstream 없음` | `savemap` 을 썼다 → `savemap-carto` 로 다시 맵핑 |
| `savemap-carto` 가 멈춤 | 터미널 A 맵핑이 켜져 있는지 (끄면 맵 사라짐) |
| `주행 가능한 영역이 없습니다` | 정제가 트랙을 지웠다 → preview 확인, 반경 낮춤 |
| `이 라인은 못 돈다` | `--apex-extra 0` 등 좁은 트랙 옵션 |
| 라인이 맵과 어긋남 | `map_name` 과 `waypoints` 가 같은 맵인지 |
| RViz 에 아무것도 안 뜸 | 같은 와이파이인지, `[dds]` 줄에 `폴백` 이 떴는지 |
| 차 위치가 엉뚱함 | `2D Pose Estimate` → `savepose` |
| 갑자기 조향 먹통 | 와이파이 바꿨으면 `./race.sh stop` 후 재시작 |
| 토픽이 도는지 모르겠음 | `cd ~/yax && ./race.sh check` |
| 편집기 창이 안 뜸 | `DISPLAY=:1` 붙였는지 (창은 젯슨 모니터에 뜸) |

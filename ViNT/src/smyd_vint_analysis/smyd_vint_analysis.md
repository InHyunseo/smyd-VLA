# smyd_vint_analysis

실행 폴더 하나를 받아 그림과 영상을 만드는 스크립트입니다. 개루프인지 폐루프인지는 폴더 안의 파일로 구분합니다.

```bash
ros2 run smyd_vint_analysis plot_run.py <실행 폴더>
ros2 run smyd_vint_analysis render_video.py <실행 폴더>
```

## plot_run.py → `figure1.png`

| 실행 | 그림 |
| --- | --- |
| 개루프 (`waypoint_errors.csv`) | 재생 시간에 따른 경유점 오차 |
| 폐루프 (`trajectory.csv`) | 왼쪽은 기록 경로와 주행 궤적, 오른쪽은 목표까지 거리와 성공 반경 |

## render_video.py → `video.mp4`, `video_overlay.mp4`

bag의 `/camera/image_raw`를 5 fps H.264로 인코딩합니다.

| 실행 | 산출물 |
| --- | --- |
| 개루프 | `video.mp4`. 재생 시각과 그 시점 경유점 오차를 프레임 위에 씁니다 |
| 폐루프 | `video.mp4`는 카메라 원본, `video_overlay.mp4`는 오른쪽에 320 px 패널을 붙여 선택된 노드 번호와 경유점을 로봇 기준 좌표로 그립니다 |

개루프는 `source_bag.txt`가 가리키는 기록 bag을 읽고, 폐루프는 실행 폴더의 `bag/`을 읽습니다. 인코딩에는 `ffmpeg`가 필요합니다.

# co_bot_2 URDF 제작 메모

생성 파일:

- `urdf/co_bot_2.urdf.xacro`
- `package.xml`
- `CMakeLists.txt`

원본 CAD:

- `C:\Users\DMUD\Desktop\co_bot_2.STEP`

확인한 STL 폴더:

- `C:\Users\DMUD\Desktop\co_bot_stl`

URDF 패키지 mesh 폴더:

- `C:\Users\DMUD\Desktop\co_bot_2_urdf\meshes`


## 1. 현재 URDF 구조

```text
base_link
├─ camera_link                       fixed
├─ lidar_link                        fixed
├─ left_wheel_link                   continuous
├─ right_wheel_link                  continuous
└─ slide_link                        prismatic, Z축 상하 이동
    ├─ left_gripper_link             prismatic, X축 좌우 이동
    │   └─ left_hand_link            fixed
    └─ right_gripper_link            prismatic, X축 좌우 이동
        └─ right_hand_link           fixed
```


## 2. 주인님이 알려준 STEP 부품 의미

| STEP 부품/바디 | URDF 링크 |
|---|---|
| `b1` | `base_link`에 포함 |
| `b2` | `base_link`에 포함 |
| `파트20^co_bot_2` body 2개 | `base_link`에 포함 |
| `파트11^co_bot_2` body | `base_link`에 포함 |
| `파트10^co_bot_2` body | `base_link`에 포함 |
| `파트24^co_bot_2` body 2개 | `base_link`에 포함 |
| `파트12^co_bot_2`의 `보스-돌출1` | `base_link`에 포함 |
| `파트19^co_bot_2` body | `base_link`에 포함 |
| `파트9^co_bot_2` | `camera_link` |
| `파트12^co_bot_2`의 `보스-돌출3` | `lidar_link` |
| `파트2^co_bot_2` 오른쪽/왼쪽 | `right_wheel_link`, `left_wheel_link` |
| `b3` | `slide_link` |
| `파트17^co_bot_2` 좌/우 | `left_gripper_link`, `right_gripper_link` |
| `파트22^co_bot_2` hand 좌/우 | `left_hand_link`, `right_hand_link` |


## 3. Prismatic 움직임 수치 바꾸는 곳

`urdf/co_bot_2.urdf.xacro` 위쪽에 있는 이 값만 바꾸면 됩니다.

```xml
<xacro:property name="slide_lower" value="0.000"/>
<xacro:property name="slide_upper" value="0.120"/>

<xacro:property name="left_gripper_lower"  value="0.000"/>
<xacro:property name="left_gripper_upper"  value="0.040"/>
<xacro:property name="right_gripper_lower" value="0.000"/>
<xacro:property name="right_gripper_upper" value="0.040"/>
```

단위는 meter입니다.

예를 들어 슬라이드가 80mm만 움직이게 하려면:

```xml
<xacro:property name="slide_upper" value="0.080"/>
```

그리퍼가 한쪽당 25mm씩 움직이게 하려면:

```xml
<xacro:property name="left_gripper_upper"  value="0.025"/>
<xacro:property name="right_gripper_upper" value="0.025"/>
```


## 4. 축 방향

현재 설정:

| joint | type | axis | 의미 |
|---|---|---|---|
| `base_to_slide_joint` | `prismatic` | `0 0 1` | Z축 위아래 |
| `slide_to_left_gripper_joint` | `prismatic` | `1 0 0` | 왼쪽 그리퍼 X축 이동 |
| `slide_to_right_gripper_joint` | `prismatic` | `-1 0 0` | 오른쪽 그리퍼 반대 X축 이동 |
| `left_wheel_joint` | `continuous` | `0 1 0` | 왼쪽 바퀴 회전 |
| `right_wheel_joint` | `continuous` | `0 1 0` | 오른쪽 바퀴 회전 |

좌우 그리퍼는 축을 서로 반대로 잡았습니다.

그래서 같은 양수 위치 명령을 주면 양쪽이 서로 반대 방향으로 벌어지는 구조로 쓰기 쉽습니다.


## 5. Mesh 파일 준비

URDF는 STEP을 직접 쓰기보다 STL/DAE/OBJ mesh를 씁니다.

`C:\Users\DMUD\Desktop\co_bot_stl`의 STL을 `meshes` 폴더로 복사하면서 이름을 정리했습니다.

현재 xacro가 사용하는 mesh 이름:

```text
meshes/body_b1.stl
meshes/body_b2.stl
meshes/body_part10.stl
meshes/body_part11.stl
meshes/body_part12_boss1.stl
meshes/body_part19.stl
meshes/body_part20_1.stl
meshes/body_part20_2.stl
meshes/body_part24_1.stl
meshes/body_part24_2.stl
meshes/camera_part9.stl
meshes/lidar_part12_boss3.stl
meshes/left_wheel_part2.stl
meshes/right_wheel_part2.stl
meshes/slide_b3.stl
meshes/left_gripper_part17.stl
meshes/right_gripper_part17.stl
meshes/left_hand_part22.stl
meshes/right_hand_part22.stl
```

주의:

- `파트12^co_bot_2`는 현재 `보스-돌출1`과 `보스-돌출3` STL이 분리되어 있습니다.
- 그래서 `보스-돌출1`은 `base_link`, `보스-돌출3`은 `lidar_link`에 따로 붙였습니다.
- 이전 초안의 `body_lidar_part12.stl`, `body_spi_lcd.stl` 파일은 mesh 폴더에 남아 있지만 현재 xacro에서는 사용하지 않습니다.


## 6. 아직 맞춰야 하는 것

현재 파일은 구조용 초안입니다.

다음 값은 실제 RViz에서 보면서 맞춰야 합니다.

- 각 mesh의 `origin xyz`
- 바퀴 회전축 방향
- 좌우 바퀴 위치
- 슬라이드 시작 위치
- 그리퍼 좌우 위치
- inertial mass/inertia 값
- collision mesh 단순화 여부


## 7. STL 좌표 확인 결과

STL은 조립 상태 좌표를 유지한 상태로 보입니다.

그래서 xacro에서는 움직이는 링크의 bbox 중심을 joint/link frame으로 잡고, visual/collision origin에는 그 중심의 음수값을 넣었습니다.

이렇게 하면 처음 표시했을 때 조립 위치가 대체로 유지되고, prismatic/continuous joint도 각 부품 중심 근처에서 움직이도록 시작할 수 있습니다.

다만 바퀴 회전축과 그리퍼 이동축은 RViz에서 확인하면서 최종 조정해야 합니다.

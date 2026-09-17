# ros2_LT

ROS 2 **Humble** 워크스페이스 (Ubuntu 22.04 / Python 3.10).
실제 하드웨어가 붙는 프로젝트다. 시뮬레이션 가정으로 코드를 쓰지 말 것.

**라인트레이서(Line Tracer)** — 바닥의 선을 센서로 읽어 따라 주행하는 로봇.

파이프라인: `센서 읽기 → 라인 위치 추정 → 조향 제어 → 모터 출력`.
각 단계는 토픽으로 분리한다. 한 노드에 다 넣으면 튜닝할 때 통째로 재시작해야 한다.

> TODO: 확정되면 채울 것 — 센서 종류(적외선 어레이 / 카메라), 모터 드라이버,
> 노드 이름과 토픽 이름.

### 라인트레이서 주의사항

- **제어 게인은 전부 파라미터다.** Kp/Ki/Kd, 기본 속도, 조향 한계는 `config/*.yaml`에서
  바꿀 수 있어야 한다. 주행하면서 튜닝하는 값이라 코드에 박으면 못 쓴다.
- **라인 로스트 처리를 먼저 짠다.** 선을 놓쳤을 때 정지할지, 마지막 조향을 유지할지
  결정하고 타임아웃을 건다. 이게 없으면 로봇이 책상에서 떨어진다.
- **센서 캘리브레이션 값은 코드가 아니라 파일에 저장한다.** 흰색/검은색 기준값은
  조명과 바닥에 따라 매번 달라진다.
- 제어 주기는 고정 타이머(`create_timer`)로 돌린다. 센서 콜백 안에서 모터를
  직접 돌리면 센서 주기에 제어 주기가 끌려간다.

## 레이아웃

```
ros2_LT/
  src/<package>/        # 여기만 편집한다
  build/ install/ log/  # colcon 생성물. 읽지도 말고 편집하지도 말 것
```

## 빌드 / 실행

쉘 상태는 명령 간에 유지되지 않는다. **매 명령마다 source를 같은 줄에 붙일 것.**

```bash
# 빌드 (워크스페이스 루트에서)
source /opt/ros/humble/setup.bash && colcon build --symlink-install

# 단일 패키지만
source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select <pkg>

# 실행
source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch <pkg> <file>.launch.py

# 테스트
source /opt/ros/humble/setup.bash && colcon test --packages-select <pkg> && colcon test-result --verbose
```

- `--symlink-install` 필수. 안 그러면 Python 수정할 때마다 재빌드해야 한다.
- 빌드 실패 시 "clean build 해볼까요"로 도망가지 말 것. 에러 원문을 읽고 원인을 고친다.
- `rm -rf build install log`는 사용자가 명시적으로 요청할 때만.

## 패키지 규칙

- Python 노드 → `ament_python`, C++ 노드 → `ament_cmake`. 한 패키지에 섞지 않는다.
- 새 패키지는 `ros2 pkg create --build-type ament_python --node-name <node> <pkg>`로 만든다. 손으로 디렉터리 만들지 말 것.
- 노드 파일 1개 = 노드 1개. `setup.py`의 `entry_points`에 등록해야 `ros2 run`이 찾는다.
- 파라미터는 `declare_parameter()`로 선언하고 기본값은 `config/*.yaml`에 둔다. 코드에 상수로 박지 말 것.
- launch는 `launch/`, 설정은 `config/`, rviz는 `rviz/`.

## 하드웨어

- 실물은 스펙대로 안 움직인다. 게인·오프셋·스케일은 **반드시 파라미터로 빼서** 튜닝 가능하게 둘 것. 코드에 상수로 박는 순간 현장에서 못 고친다.
- 모터/구동 명령을 내보내는 코드에는 정지 경로(커맨드 타임아웃, 출력 범위 클램프)를 먼저 넣는다.
- 장치 노드 경로(`/dev/ttyUSB0` 등)는 파라미터로. 부팅 순서에 따라 바뀐다.
- 디바이스 확인: `ls -l /dev/tty*`, `lsusb`, `i2cdetect -y 1`

## 토픽 / QoS

- 센서 스트림(라이다, 카메라, IMU)은 `rclpy.qos.qos_profile_sensor_data`. 기본 RELIABLE을 쓰면 드랍 대신 지연이 쌓인다.
- 상태/명령 토픽은 기본 QoS로 충분하다.
- 디버깅 순서: `ros2 topic list` → `ros2 topic hz <t>` → `ros2 topic echo <t> --once`.
  코드를 읽기 전에 실제로 뭐가 흐르는지 먼저 본다.

## 코드 스타일

- 게으르게. 이미 있는 걸 다시 만들지 않는다. 표준 메시지(`geometry_msgs`, `sensor_msgs`)로 표현되면 커스텀 msg를 만들지 않는다.
- 추상화는 구현체가 2개 이상 생긴 뒤에 만든다.
- 분기·루프·파싱이 들어간 로직에는 실행 가능한 체크를 하나 남긴다 (`test_*.py` 또는 `__main__` 자가 검증).

## 하지 말 것

- `build/`, `install/`, `log/` 편집
- `sudo` 자동 실행 — 필요하면 사용자에게 명령을 알려주고 직접 실행하게 한다
- 하드웨어가 연결되지 않은 상태에서 "동작 확인됨"이라고 보고하기

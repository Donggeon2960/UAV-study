# UAV 3주차 강의안 (V2 - 2025년 트렌드 강화판)
## 임베디드시스템 SW 개발 방법론: UAM 및 고신뢰성 드론 개발 실무

**대상**: 컴퓨터공학과 3학년 (임베디드 관심 학생)
**목표**: 개발 방법론 중심 (개발환경, 빌드, 디버깅, 테스트, 최신 산업 트렌드)
**분량**: 52 슬라이드 + @ (최신 트렌드 추가 슬라이드)

---

## 🌟 2025년 기준 핵심 추가 사항 (V2)

본 강의안은 원본(`UAV_Week3_Complete_Lecture.md`)에 `UAM_drone_embedded_system_additions.md` 및 최신 산업/연구 트렌드를 통합하였습니다.

**주요 강화 포인트:**
* **산업 표준 인증 (MBD):** 모델 기반 설계(Simulink) 및 DO-178C 인증 연계
* **개발/운영 자동화 (DevOps):** CI/CD 파이프라인 및 Docker 환경 표준화
* **차세대 자율비행 (AI/ROS):** ROS 2 통합, 온보드 AI(VIO, 엣지 컴퓨팅)
* **고급 시뮬레이션 (Digital Twin):** SITL을 넘어선 디지털 트윈 개념
* **보안 및 신뢰성 (Security):** GPS 스푸핑 방어, MAVLink 서명 등 실전 보안

---

## 📑 전체 목차 (V2)

- Part 1: 비행제어 소프트웨어 아키텍처 (슬라이드 1-12)
- Part 2: 비행제어 알고리즘 & 코드 (슬라이드 13-23)
- Part 3: 개발 환경 구축 (슬라이드 24-31)
- Part 4: 펌웨어 빌드 & 배포 (+ CI/CD) (슬라이드 32-41)
- Part 5: SITL & 실전 워크플로우 (+ MBD, ROS, AI, Security) (슬라이드 42-52)

---

# Part 1: 비행제어 소프트웨어 아키텍처

*(슬라이드 1-12는 원본과 동일하게 유지합니다. 아키텍처의 기본은 견고합니다.)*

## 슬라이드 1-2: 표지 및 목차
- 제목: UAV 임베디드시스템 SW 개발 방법론 (V2)
- 학습 목표: 소프트웨어 구조 이해, 알고리즘 분석, 개발환경 구축, 빌드/배포, SITL, **최신 산업 트렌드(MBD, CI/CD, AI, Security)**

## 슬라이드 3: 오픈소스 생태계

| 항목 | PX4 Autopilot | ArDuPilot |
|------|---------------|-----------|
| RTOS | NuttX | ChibiOS |
| 언어 | C++14 | C++11 |
| 코드 | ~300k LOC | ~700k LOC |
| 라이선스 | BSD | GPL v3 |
| GitHub | PX4/PX4-Autopilot | ArduPilot/ardupilot |

## 슬라이드 4: PX4 4계층 아키텍처

```
┌─────────────────────────────────────┐
│  Application Layer                  │ ← EKF2, 제어기, 항법
├─────────────────────────────────────┤
│  Middleware Layer                   │ ← uORB, MAVLink, Logger
├─────────────────────────────────────┤
│  OS Layer (NuttX RTOS)              │ ← 스케줄링, 드라이버
├─────────────────────────────────────┤
│  Hardware (Pixhawk STM32)           │ ← ARM Cortex-M7
└─────────────────────────────────────┘
```

**장점**: 유지보수성, 재사용성, 테스트 용이

## 슬라이드 5: ArduPilot 구조
- 차량별 분리: ArduCopter, ArduPlane, ArduRover
- 공유 라이브러리: AP_AHRS, AP_Motors, EKF3, AC_PID
- HAL 추상화: ChibiOS, Linux 지원

## 슬라이드 6: uORB 메시지 버스
**Publish-Subscribe 패턴**

발행 예시:
```cpp
orb_advert_t accel_pub = orb_advertise(ORB_ID(sensor_accel), &data);
orb_publish(ORB_ID(sensor_accel), accel_pub, &accel_data);
```

구독 예시:
```cpp
int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
orb_copy(ORB_ID(sensor_accel), accel_sub, &accel_data);
```

## 슬라이드 7: RTOS 실시간 스케줄링

**왜 RTOS?**
- 일반 OS: "언젠가" 실행 (수십 ms 지연)
- RTOS: "정확히 X ms 후" 실행 보장

**우선순위 예시**:
- 250: 센서 인터럽트
- 200: 각속도 제어 (1000Hz)
- 180: 자세 제어 (250Hz)
- 160: 위치 제어 (50Hz)

## 슬라이드 8: 메모리 관리

```
Flash ROM (2MB):  펌웨어 저장
SRAM (512KB):     런타임 데이터
CCM RAM (128KB):  제어 루프 전용 (0 wait state)
```

**CCM의 특별함**: CPU 직접 접근, DMA 불가, 초고속

## 슬라이드 9: 드라이버 구조

MPU6000 IMU 드라이버 예시:

```cpp
void MPU6000::Run() {
    // 1. SPI로 센서 읽기 (1000Hz)
    read_registers(MPUREG_ACCEL_XOUT_H, data, 14);
    
    // 2. Raw → 물리량 변환
    accel_x = (int16_t)((data[0] << 8) | data[1]) * scale;
    
    // 3. uORB 발행
    orb_publish(ORB_ID(sensor_accel), _pub, &accel_report);
}
```

## 슬라이드 10: 소프트웨어 실행 흐름

부팅 → NuttX → PX4 앱 → 모듈 시작 → Armed → 비행

주요 모듈:
- sensors (1000Hz)
- ekf2 (250Hz)
- mc_rate_control (1000Hz)
- mc_att_control (250Hz)
- mc_pos_control (50Hz)

## 슬라이드 11: 모듈 간 상호작용

RC 조종기 → Commander → Navigator → Position → Attitude → Rate → Mixer → PWM → ESC

## 슬라이드 12: Part 1 요약

핵심: 계층적 아키텍처, uORB, RTOS, 메모리, 드라이버, 실행 흐름

---

# Part 2: 비행제어 알고리즘 & 코드

## 슬라이드 13: PID 제어 이론

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

- P: 현재 오차에 즉각 반응
- I: 누적 오차 제거 (정상상태)
- D: 오차 변화율로 제동 (오버슈트 감소)

## 슬라이드 14: 계단식 제어 구조

4단계:
1. Level 4: 위치 제어 (50Hz) → 속도 명령
2. Level 3: 속도 제어 (50Hz) → 자세 명령
3. Level 2: 자세 제어 (250Hz) → 각속도 명령
4. Level 1: 각속도 제어 (1000Hz) → 모터 출력

**Rule**: 내부 루프는 외부보다 5-10배 빠르게

## 슬라이드 15: 각속도 제어기 (Rate Controller)

**PX4 코드**:

```cpp
class RateControl {
    float update(float rate_sp, float rate, float dt) {
        float error = rate_sp - rate;
        // P, I, D 항 계산...
        float output = P + _integral + D;
        return constrain(output, -1.0f, 1.0f);
    }
};
```

**전형적인 게인**: Kp: 0.15, Ki: 0.05, Kd: 0.003

## 슬라이드 16: 자세 제어기 (Attitude Controller)

250Hz, 쿼터니언 → 각속도 명령

```cpp
Vector3f rate_sp;
Quatf q_error = q.inversed() * q_sp;  // 자세 오차
Vector3f e_R = q_error.to_axis_angle();
rate_sp = e_R.emult(_gain);  // P 제어
```

**Kp 값**: Roll 6.5, Pitch 6.5, Yaw 2.8

## 슬라이드 17: 위치/속도 제어기

50Hz, GPS 기반

```cpp
// 위치 → 속도
Vector3f pos_error = _pos_sp - _pos;
_vel_sp = _gain_pos_p.emult(pos_error);

// 속도 → 자세
// "앞으로 가속하려면 얼마나 기울여야?"
float pitch_sp = -asin(accel_x / thrust.length());
```

## 슬라이드 18: PID 코드 통합 예시

```cpp
class PIDController {
    float update(float setpoint, float measurement) {
        // P, I, D 항 계산...
        return constrain(p_term + i_term + d_term, -1.0f, 1.0f);
    }
};
```

---

## ⭐ 추가 슬라이드 18-1: 개발 방법론 (1) - 모델 기반 설계 (MBD)

**"UAM/eVTOL 같은 고신뢰성 시스템은 이 코드를 손으로 짜지 않습니다."**

**모델 기반 설계 (Model-Based Design)**

* **개념:** 제어 알고리즘(PID, EKF)을 C++ 코드가 아닌 **그래픽 모델(블록 다이어그램)**로 설계
* **도구:** MATLAB/Simulink, (MathWorks)
* **프로세스:**
  1. **설계:** Simulink에서 PID 제어기를 블록으로 설계
  2. **시뮬레이션 (SIL/HIL):** 모델을 시뮬레이션하여 검증
  3. **코드 생성:** **Simulink Coder**가 검증된 모델로부터 C/C++ 코드를 **자동 생성**
* **왜? (DO-178C):**
    * **오류 감소:** 사람이 직접 코딩 시 발생하는 휴먼 에러 원천 차단
    * **추적성:** 요구사항-설계-모델-코드-검증 전 과정이 추적/관리됨
    * **DO-178C (항공 SW 인증)**의 핵심 요구사항 충족

---

## 슬라이드 19: EKF 센서 융합

**왜 필요?**
- 자이로: 빠르지만 Drift
- 가속도: 중력 vs 가속도 구분 못함
- GPS: 느림 (5Hz)
- 지자기: 전자기 간섭

**EKF 2단계**:
1. 예측 (1000Hz): IMU로 상태 예측
2. 업데이트 (5-100Hz): GPS/Mag/Baro로 보정

## 슬라이드 20: EKF 수식

```
예측:
x̂(k|k-1) = F·x̂(k-1) + B·u(k)
P(k|k-1) = F·P(k-1)·F^T + Q

업데이트:
K = P·H^T·(H·P·H^T + R)^-1
x̂(k|k) = x̂(k|k-1) + K·(z - H·x̂(k|k-1))
```

칼만 게인 K: 예측 vs 측정의 신뢰도

## 슬라이드 21: EKF 코드 구조

```cpp
void Ekf2::Run() {
    // 1. IMU 예측 (1000Hz)
    _ekf.predictState(imu);
    
    // 2. GPS 업데이트 (5Hz)
    if (gps_updated) {
        _ekf.fuseGps(gps);
    }
    // ... (Mag, Baro)
    
    // 4. 결과 발행 (250Hz)
    _ekf.getQuaternion(att.q);
    orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
}
```

---

## ⭐ 추가 슬라이드 21-1: 차세대 항법 - 온보드 AI (VIO / AI 융합)

**"GPS가 끊기거나 교란되는 도심(UAM) 환경에서는 EKF만으로 부족합니다."**

**1. Visual-Inertial Odometry (VIO)**

* **개념:** 카메라 영상(Visual)과 IMU(Inertial)를 융합하여 GPS 없이 실시간 위치/자세 추정
* **활용:** GPS 음영 지역(실내, 도심 협곡)에서의 정밀 항법 (Skydio 드론)
* **개발:** FCU(Pixhawk)가 아닌, **컴패니언 컴퓨터(NVIDIA Jetson, ModalAI VOXL)**에서 VIO 연산 후, MAVLink로 FCU에 위치값 전송

**2. AI 기반 센서 융합 / 장애물 회피**

* **센서 융합:** EKF를 보완/대체. 비정형 노이즈(바람, 진동)에 강인한 신경망 기반 필터링
* **장애물 회피:** Depth 카메라/LiDAR 데이터를 AI가 실시간 분석하여 회피 경로 생성 (단순 정지 X)
* **개발:** TensorFlow Lite, ONNX Runtime 등으로 AI 모델을 경량화하여 엣지(컴패니언 컴퓨터)에서 실행

---

## 슬라이드 22: 튜닝 방법론

**Step-by-Step**:
1. 시뮬레이터에서 시작
2. 각속도 루프 튜닝 (Kp → Kd → Ki)
3. 자세 루프 튜닝
4. 위치 루프 튜닝

**Ziegler-Nichols 방법**:
- Ku 찾기 (지속 진동)
- Kp = 0.6*Ku, Ki = 2*Kp/Tu, Kd = Kp*Tu/8

---

## ⭐ 추가 슬라이드 22-1: 개발 방법론 (2) - 자동 PID 튜닝

**"수동 튜닝은 어렵고 시간이 많이 소요됩니다."**

**1. QGroundControl 오토-튜닝**

* QGroundControl -> Vehicle Setup -> PID Tuning
* 특정 축(Roll/Pitch)을 흔드는 명령을 자동으로 수행하며 최적의 P/I/D 게인 탐색
* 실습: (슬라이드 22의 수동 튜닝 후) 자동 튜닝을 실행하여 값 비교

**2. ArduPilot Quicktune**

* 풍속 등 외부 요인을 고려한 자동 튜닝 알고리즘

**3. 연구 트렌드: ML/RL 기반 튜닝**

* 강화학습(RL) 에이전트가 수천 번의 시뮬레이션을 통해 최적의 PID 값을 찾는 차세대 방식

---

## 슬라이드 23: Part 2 요약

핵심: PID, 계단식 제어, 각속도/자세/위치 제어기, EKF, 튜닝
**+ (산업)** MBD, (차세대) 온보드 AI/VIO, (실무) 자동 튜닝

---

# Part 3: 개발 환경 구축

## 슬라이드 24: 개발 환경 개요

필요한 것:
1. Ubuntu 22.04
2. ARM GCC 툴체인
3. VSCode
4. Gazebo
5. QGroundControl

## 슬라이드 25: Ubuntu 설치

**방법**:
- 듀얼 부팅 (권장)
- VirtualBox/VMware
- WSL2 (Windows Subsystem for Linux 2)

**권장 사양**: i5/Ryzen 5, 16GB RAM, 100GB SSD

## 슬라이드 26: PX4 개발환경 구축

```bash
# 원클릭 설치
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 첫 빌드
make px4_sitl
```

**디렉토리 구조**: src/, boards/, build/

## 슬라이드 27: ArduPilot 개발환경

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 빌드
cd ArduCopter
./waf configure --board px4-v2
./waf copter
```

## 슬라이드 28: VSCode 설정

**필수 확장**:
- C/C++ (Microsoft)
- Python
- CMake Tools
- GitLens
- Cortex-Debug

## 슬라이드 29: Git 워크플로우

```bash
# 브랜치 생성
git checkout -b feature/custom-pid

# 코드 수정
vim src/modules/mc_rate_control/RateControl.cpp

# 커밋
git add .
git commit -m "Improve PID gains"

# 푸시
git push origin feature/custom-pid
```

## 슬라이드 30: 디버깅 도구

1. **printf**: PX4_INFO(), PX4_WARN(), PX4_ERR()
2. **uORB 모니터**: `listener sensor_accel`
3. **MAVLink Inspector**: QGC
4. **로그**: Flight Review (logs.px4.io)
5. **GDB**: SITL 디버깅
6. **JTAG**: 하드웨어 디버깅

---

## ⭐ 추가 슬라이드 30-1: 고급 디버깅 및 성능 분석

**"printf 디버깅만으로는 실시간 시스템의 문제를 잡기 어렵습니다."**

**1. 실시간 프로파일링 (NSH)**

* `top`: NuttX의 실시간 작업(Task) CPU 점유율 확인
* `uorb top`: uORB 메시지 발행/구독 상태, 지연(delay) 확인

**2. 성능 분석 (Linux/SITL)**

* **perf:** Linux 커널 수준의 프로파일러. 어떤 함수가 CPU를 많이 쓰는지 분석
* **valgrind:** SITL 환경에서 메모리 누수(leak)나 잘못된 접근 검사

**3. 타이밍 분석 (JTAG/Trace)**

* **Tracealyzer:** JTAG 디버거와 연동하여 RTOS 태스크 스케줄링, 인터럽트, uORB 메시지 흐름을 시각화
* "왜 제어 루프가 4ms가 아닌 5ms 만에 돌았는가?"를 마이크로초(µs) 단위로 분석

---

## 슬라이드 31: Part 3 요약

체크리스트:
- ✅ Ubuntu 설치
- ✅ PX4 소스 다운로드
- ✅ ARM GCC 툴체인
- ✅ VSCode 설정
- ✅ 디버깅 도구 **( + 고급 프로파일링)**

---

# Part 4: 펌웨어 빌드 & 배포

## 슬라이드 32: 빌드 시스템

**PX4: CMake**

```bash
make px4_sitl          # 시뮬레이터
make px4_fmu-v5        # Pixhawk 4
make px4_fmu-v2        # Pixhawk 2.4.8 (Blueye-1K)
```

**ArduPilot: Waf**

```bash
./waf configure --board px4-v2
./waf copter
```

## 슬라이드 33: PX4 빌드 명령어

```bash
make px4_fmu-v5
make px4_fmu-v5 -j8  # 병렬 빌드
make clean           # 클린
make px4_fmu-v5 upload # 빌드 + 업로드
```

## 슬라이드 34: ArduPilot 빌드

```bash
./waf configure --board px4-v2
./waf copter
./waf copter --upload
```

## 슬라이드 35: 펌웨어 업로드

**방법 1: USB** (`make ... upload`)
**방법 2: QGroundControl** (Custom .px4 파일 선택)

## 슬라이드 36: 시리얼 통신 (NSH Console)

```bash
screen /dev/ttyACM0 57600

# 명령어
ver all
ps
top
free
listener sensor_accel
param show MC_ROLLRATE_P
```

## 슬라이드 37: MAVLink 통신

**Python 예시**:

```python
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()

msg = master.recv_match(type='ATTITUDE', blocking=True)
print(f"Roll: {msg.roll}, Pitch: {msg.pitch}")

master.arducopter_arm()
```

## 슬라이드 38: 파라미터 설정

**QGC**: Vehicle Setup → Parameters
**NSH**: `param set MC_ROLLRATE_P 0.2` → `param save`
**코드**: `param_get(_params_handles.roll_rate_p, &_params.roll_rate_p);`

## 슬라이드 39: 로그 분석

**수집**: SD 카드 (*.ulg) 또는 QGC Log Download
**분석**:
- Flight Review: https://logs.px4.io
- Python: ulog2csv → pandas

## 슬라이드 40: 문제 해결

- 빌드 실패 → 의존성, 서브모듈 확인
- 업로드 실패 → 포트 권한, 부트로더 확인
- ARM 불가 → 센서 캘리브레이션, GPS 확인

## 슬라이드 41: Part 4 요약

핵심: 빌드 (Make/Waf), 업로드, 시리얼 통신, MAVLink, 파라미터, 로그

---

## ⭐ 추가 슬라이드 41-1: 개발 방법론 (3) - CI/CD 파이프라인

**"대규모 UAM/드론 SW는 'make' 명령어로 개발하지 않습니다."**

**CI (Continuous Integration):**

* **개념:** 개발자가 코드를 Git에 `push` 할 때마다, 서버가 자동으로 **빌드** 및 **단위 테스트**를 수행
* **장점:**
    * 빌드 오류 즉시 발견 (ARM 펌웨어를 빌드했는데 x86 코드가 들어감)
    * SITL 자동 테스트로 알고리즘 버그 조기 발견

**CD (Continuous Deployment):**

* **개념:** CI가 성공하면, 빌드된 펌웨어(.px4)를 **자동으로 배포** (테스트 기체 또는 시뮬레이션 환경에)
* **도구:** **GitHub Actions**, GitLab CI, Jenkins

## ⭐ 추가 슬라이드 41-2: CI/CD 예시 (GitLab CI)

`.gitlab-ci.yml` 예시:

```yaml
stages:
  - build      # 1. 펌웨어 빌드
  - test       # 2. SITL 자동 테스트

build_px4_fmu-v5:
  stage: build
  image: px4io/px4-dev-nuttx:latest  # Docker 이미지 사용
  script:
    - make px4_fmu-v5_default
  artifacts:
    paths:
      - build/px4_fmu-v5_default/*.px4 # 빌드 산출물 저장

test_sitl_mission:
  stage: test
  image: px4io/px4-dev-simulation:latest
  script:
    - make px4_sitl_default gazebo
    - python3 ./Tools/run_mission_test.py # 자동 미션 테스트
```

## ⭐ 추가 슬라이드 41-3: 개발 환경 표준화 (Docker)

**"팀원의 PC 환경이 달라 빌드가 안 되는 문제를 해결합니다."**

* **문제:** "제 PC에선 되는데요?" (Ubuntu 20.04 vs 22.04, Python 3.8 vs 3.10)
* **해결 (Docker):**
    * PX4/ArduPilot 개발에 필요한 **모든 툴체인(ARM GCC, Python 등)이 설치된 Docker 이미지**를 사용
    * 팀원 모두가 동일한 개발/빌드 환경을 가짐
    * 위 `gitlab-ci.yml`의 `image: px4io/px4-dev-nuttx:latest`가 그 예시

---

# Part 5: SITL & 실전 워크플로우

## 슬라이드 42: SITL 개념

**Software In The Loop**

* 실제 펌웨어 코드(PX4)를 수정 없이 PC에서 실행
* Gazebo(시뮬레이터)가 가상의 센서 값(IMU, GPS)을 펌웨어에 전달
* 펌웨어가 계산한 모터 값(PWM)을 Gazebo가 받아 기체 이동

## 슬라이드 43: Gazebo 시뮬레이터

```bash
# 기본 실행
make px4_sitl gazebo

# 특정 모델
make px4_sitl gazebo_x500
make px4_sitl gazebo_vtol

# 헤드리스 (서버용)
HEADLESS=1 make px4_sitl gazebo
```

## 슬라이드 44: SITL 미션 (Python MAVSDK)

```python
import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(10)
    await drone.action.land()

asyncio.run(run())
```

## 슬라이드 45: 디버깅 & 테스트

```bash
# GDB 디버깅
make px4_sitl gazebo___gdb

(gdb) break MulticopterRateControl::Run
(gdb) print error
(gdb) continue

# 단위 테스트
make tests

# 통합 테스트
make px4_sitl_default gazebo mavsdk_tests
```

## 슬라이드 46: SITL vs 실제

| 항목 | SITL | 실제 |
|------|------|------|
| 센서 노이즈 | 작음 | 큼 |
| 진동 | 없음 | 많음 |
| GPS 지연 | 없음 | 100-200ms |
| 추락 | 리셋 | 파손 💥 |

**전환 팁**: SITL → 벤치 테스트 → 실내 호버링 → 실외 비행

---

## ⭐ 추가 슬라이드 46-1: SITL을 넘어서 - 디지털 트윈 (Digital Twin)

**"SITL은 개발용, 디지털 트윈은 개발 및 운영(UAM)용입니다."**

* **SITL (개발):** 개발자가 *가상 환경*에서 *가상 드론*을 테스트
* **Digital Twin (운영):**
    * **개념:** *현실의 특정 UAM 기체* (예: '기체 A')와 1:1로 매핑되는 *가상 복제본*
    * **동기화:** '기체 A'의 실제 센서 데이터, 로그, 운용 이력이 가상 복제본에 **실시간 동기화**됨
    * **활용 (UAM):**
      1. **고장 예측:** '기체 A'가 실제 고장나기 전, 디지털 트윈에서 먼저 징후(예: 모터 진동 증가)를 발견
      2. **관제 시뮬레이션:** 수천 대의 UAM 디지털 트윈을 동시에 띄워 복잡한 도심 관제 시나리오 검증

---

## 슬라이드 47: 실전 개발 프로세스

```
요구사항 정의
  ↓
설계 (알고리즘, 코드 구조)
  ↓
구현 (VSCode)
  ↓
단위 테스트
  ↓
SITL 테스트
  ↓
코드 리뷰
  ↓
하드웨어 테스트
  ↓
튜닝 & 최적화
  ↓
문서화
  ↓
배포
```

---

## ⭐ 강화 슬라이드 47-1: 실전 개발 프로세스 (MBD/CI/CD 적용)

```
요구사항 정의
  ↓
[MBD] 알고리즘 모델링 (Simulink)
  ↓
[MBD] 모델 시뮬레이션 (SIL)
  ↓
[MBD] 자동 코드 생성 (C/C++)
  ↓
[통합] 생성된 코드 + 직접 작성한 코드 (VSCode)
  ↓
[Git] 'feature' 브랜치에 Push
  ↓
[CI] 자동 빌드 (Docker)
  ↓
[CI] 자동 단위 테스트 & SITL 테스트
  ↓
[Review] 코드 리뷰 (GitHub)
  ↓
[CD] 'main' 브랜치 병합 → 테스트 기체에 자동 배포
  ↓
하드웨어 테스트 & 튜닝
  ↓
배포
```

---

## 슬라이드 48: 코드 수정 예시

**시나리오**: Roll Rate P 게인 증가

```bash
# 1. 파일 찾기 (파라미터 정의)
grep -r "MC_ROLLRATE_P" src/

# 2. 수정 (src/modules/mc_rate_control/mc_rate_control_params.c)
# 3. 빌드 & 테스트
make px4_sitl gazebo

# 4. Git commit
git commit -m "Increase MC_ROLLRATE_P to 0.20"
```

---

## ⭐ 추가 슬라이드 48-1: 개발 방법론 (4) - ROS 2 통합

**"단순 비행을 넘어, 자율 임무(SLAM, 회피, 멀티 드론)를 수행합니다."**

* **PX4 (FCU):** 실시간 비행 제어 (1000Hz PID)
* **ROS 2 (컴패니언 컴퓨터):** 무거운 연산 (SLAM, AI, 경로 계획) (10-50Hz)
* **연결:** **MicroXRCE-DDS Bridge** (초고속, 저지연 통신)

**실습:**

1. **PX4/ROS 2 브리지 활성화**
    ```bash
    # PX4 측
    make px4_sitl_rtps gazebo
    ```
2. **ROS 2 측 (컴패니언 컴퓨터)**
    ```bash
    # px4_ros_com 설치
    colcon build --packages-select px4_ros_com

    # ROS 2 토픽으로 PX4 센서 데이터(IMU) 확인
    ros2 topic echo /fmu/out/sensor_imu
    ```

## ⭐ 추가 슬라이드 48-2: ROS 2 + 온보드 AI (최종 아키텍처)

* **PX4 (FCU):** Rate, Attitude 제어 (NuttX)
* **컴패니언 컴퓨터 (NVIDIA Jetson):**
    * **ROS 2:** 노드 간 통신, 미션 관리
    * **VIO/SLAM:** GPS 없이 위치 추정
    * **AI (TensorRT):** 장애물 인식, 경로 계획
    * **MAVLink/DDS:** PX4에 '위치/속도/자세' 명령 하달

---

## 슬라이드 49: 커스텀 모듈

**새 모듈 생성**:

```bash
mkdir src/modules/my_controller
cd src/modules/my_controller

# my_controller.cpp 작성
# CMakeLists.txt 작성
# boards/px4/fmu-v5/default.px4board에 추가

make px4_sitl
nsh> my_controller start
```

## 슬라이드 50: 안전 고려사항

**원칙**:
1. Failsafe 활성화 (배터리, RC, GPS)
2. 지오펜스 설정
3. 안전 스위치 사용
4. 단계적 테스트 (SITL → 벤치 → 실내 → 실외)

**코드 안전**:
- 타임아웃 설정
- 배열 범위 체크
- NULL 포인터 체크
- 입력 검증

---

## ⭐ 추가 슬라이드 50-1: 실전 보안 및 Failsafe (UAM/eVTOL)

**"Failsafe는 기능이 아니라, 인증(DO-178C)의 핵심입니다."**

**1. GPS 스푸핑(Spoofing) 방어 (SW 개발)**

* **위협:** 해커가 가짜 GPS 신호를 주입하여 드론 탈취
* **방어 로직 (개발):**
  1. `if (gps_updated)`를 무조건 신뢰하지 않음
  2. **교차 검증:** GPS가 보고한 위치 변화와 IMU(가속도계)가 측정한 물리적 가속도를 실시간 비교
  3. **Failsafe:** 두 값이 크게 어긋나면 GPS 신뢰도를 낮추고, VIO 또는 IMU 기반 비행 모드로 즉시 전환

**2. MAVLink 2 메시지 서명 (보안 설정)**

* **위협:** MAVLink 통신(GCS-드론)을 가로채 가짜 제어 명령(Land, RTL) 주입
* **방어 (설정):**
  1. MAVLink 2 **메시지 서명(Authentication)** 활성화
  2. GCS와 드론이 **사전 공유된 비밀 키(Secret Key)**로 모든 패킷에 서명
  3. 서명이 없거나 유효하지 않은 패킷은 FCU 펌웨어 레벨에서 폐기

**3. UAM 인증 표준 (DO-178C)**

* `additions.md` 및 'MBD' 슬라이드에서 언급됨
* 소프트웨어의 모든 요구사항, 설계, 코드가 **100% 검증되고 추적 가능**해야 함을 의미

---

## 슬라이드 51: Part 5 요약

핵심: SITL, Gazebo, Python 미션, 개발 프로세스, 안전
**+ (산업)** Digital Twin, CI/CD, ROS 2, AI/VIO, 실전 보안

## 슬라이드 52: 전체 강의 요약

**학습 성과**:
- ✅ PX4/ArduPilot 소스코드 이해
- ✅ 제어 알고리즘 분석/수정
- ✅ 개발환경 구축 및 빌드
- ✅ SITL 테스트
- ✅ 하드웨어 배포
- ✅ **(산업)** MBD, CI/CD, ROS 2, AI, 보안 등 최신 개발 방법론 이해

**다음 단계**:
- 1-2주: Ubuntu 설치, PX4 빌드, SITL 실행
- 1개월: 소스코드 분석, 파라미터 튜닝
- 3개월: **커스텀 모듈, ROS 2 연동, CI 파이프라인 구축**

**추가 자료**:
- PX4 Dev Guide, ArduPilot Dev Wiki, ROS 2 Docs

---

# 부록: 실습 과제 (V2 강화)

## 과제 1: 개발환경 구축 (필수)

- Ubuntu 설치, PX4 소스 클론 및 빌드, Gazebo SITL 실행

## 과제 2: 소스코드 분석 (필수)

- Rate Controller 코드(mc_rate_control) 읽기, 주요 함수 흐름 분석

## 과제 3: SITL 미션 - MAVSDK (필수)

- Python(MAVSDK)으로 정사각형 비행 미션, Flight Review 로그 제출

## 과제 4: 파라미터 튜닝 (선택)

- P 게인 3가지 값으로 실험, 로그 비교 분석

## ⭐ 과제 5: 커스텀 모듈 (도전)

- `vibration_monitor` 모듈 개발 (IMU 진동 감지 및 경고)

## ⭐ 과제 6: CI/CD 파이프라인 (도전 - 현업 수준)

- 본인 GitHub 계정에 PX4 포크(Fork)
- GitHub Actions 연동
- `push` 시 `px4_fmu-v5` 펌웨어가 자동으로 빌드되고, 'Artifacts'로 업로드되도록 `.yml` 파일 작성

## ⭐ 과제 7: ROS 2 연동 (도전 - R&D 수준)

- `px4_sitl_rtps` 실행
- ROS 2 노드를 작성하여 `/fmu/out/sensor_imu` 토픽을 구독
- IMU 데이터를 터미널에 출력

---

# Week 4 Preview: Ground Control Station (GCS) 프로그램

## ⭐ 슬라이드 53: GCS 개요

**Ground Control Station (GCS)란?**

* **정의:** UAV를 원격으로 제어하고 모니터링하는 지상 소프트웨어
* **역할:**
  1. **임무 계획 (Mission Planning):** 지도 위 Waypoint 설정, 자동 비행 경로 생성
  2. **실시간 모니터링:** 기체 자세, 고도, 속도, GPS, 배터리 등 텔레메트리 시각화
  3. **펌웨어 관리:** 비행 제어기 펌웨어 업로드, 파라미터 설정
  4. **로그 분석:** 비행 후 데이터 분석 및 문제 진단
  5. **영상 수신:** FPV 카메라 영상 실시간 스트리밍

* **통신 프로토콜:** **MAVLink** (Micro Air Vehicle Link)
  - PX4, ArduPilot과의 표준 통신 프로토콜
  - UDP, TCP, Serial 통신 지원

---

## ⭐ 슬라이드 54: 주요 오픈소스 GCS 비교 (2025년 기준)

| GCS | 플랫폼 | 펌웨어 지원 | 최신 버전 | 특징 |
|-----|--------|------------|-----------|------|
| **QGroundControl** | Windows, Mac, Linux, Android, iOS | PX4, ArduPilot | v5.0.8 (2025.10) | • 크로스 플랫폼<br>• 직관적 UI<br>• PX4 공식 GCS |
| **Mission Planner** | Windows (주), Linux(Mono), Android | ArduPilot | v1.3.82 (2024.07) | • 가장 많은 기능<br>• ArduPilot 공식 GCS<br>• SITL 통합 |
| **APM Planner 2.0** | Windows, Mac, Linux | ArduPilot | - | • QGC 기반<br>• Mission Planner 대안 |
| **MAVProxy** | Windows, Mac, Linux | PX4, ArduPilot | - | • CLI 기반<br>• 스크립트 자동화<br>• 네트워킹 |
| **Tower** | Android | ArduPilot | - | • 모바일 전용<br>• Follow-me, Dronies |

**선택 가이드:**
- **초보자 + 크로스 플랫폼:** QGroundControl
- **ArduPilot + 고급 기능:** Mission Planner
- **모바일:** QGC (iOS/Android) 또는 Tower (Android)
- **개발/자동화:** MAVProxy

---

## ⭐ 슬라이드 55: QGroundControl (QGC) 상세

**개발:** MAVLink Project (PX4 기반)
**최신 버전:** v5.0.8 (2025년 10월)
**라이선스:** Apache 2.0 / GPLv3

**핵심 기능:**

1. **직관적 UI:**
   - Plan View: 드래그 앤 드롭 미션 계획
   - Fly View: 실시간 HUD, 지도, 텔레메트리
   - Analyze View: 로그 분석, 그래프
   - Vehicle Setup: 센서 캘리브레이션, 파라미터 설정

2. **멀티 플랫폼:**
   - 데스크톱: Windows 11/10, macOS, Linux
   - 모바일: Android, iOS
   - 동일한 UX/UI

3. **고급 기능:**
   - 비디오 스트리밍 (UDP/RTSP)
   - 지오펜스 (Geofence) 설정
   - Rally Point 설정
   - Survey 미션 (지역 스캔)
   - 다중 기체 동시 제어

**다운로드:** https://docs.qgroundcontrol.com

---

## ⭐ 슬라이드 56: Mission Planner 상세

**개발:** Michael Oborne (ArduPilot)
**최신 버전:** v1.3.82 (2024년 7월)
**플랫폼:** Windows (네이티브), Linux (Mono), Android (베타)
**라이선스:** GPL v3

**핵심 기능:**

1. **가장 풍부한 기능셋:**
   - 임무 계획 (Flight Planner)
   - 실시간 비행 데이터 (Flight Data)
   - 펌웨어 업로드 (Initial Setup)
   - 시뮬레이션 (SITL 통합)
   - 로그 분석 (DataFlash Logs → Flight Review)
   - 파라미터 비교/편집

2. **전용 기능:**
   - AutoTune (자동 PID 튜닝)
   - Antenna Tracker 지원
   - Custom Scripts (Python)
   - 3D 미션 뷰어

3. **설치:**
   ```bash
   # Windows
   다운로드: https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi
   
   # Linux (Mono 필요)
   sudo apt install mono-complete
   wget https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.zip
   unzip MissionPlanner-latest.zip
   mono MissionPlanner.exe
   ```

**장점:** ArduPilot에 최적화, 모든 기능 지원
**단점:** Windows 중심, UI 복잡도 높음

---

## ⭐ 슬라이드 57: GCS 실습 - QGroundControl

**시나리오:** SITL과 QGC 연결 → 미션 계획 → 자동 비행

**1. PX4 SITL 실행 (터미널 1)**
```bash
cd PX4-Autopilot
make px4_sitl gazebo
# MAVLink UDP: 14540 포트로 데이터 송출
```

**2. QGroundControl 실행**
- QGC 자동으로 UDP:14540 연결
- "Vehicle Connected" 메시지 확인

**3. 미션 계획 (Plan View)**
- 지도 클릭 → Waypoint 추가 (최소 3개)
- Waypoint 타입 설정:
  - Takeoff (10m)
  - Waypoint (이동 지점)
  - Land (착륙)
- "Upload" 버튼 → 기체에 미션 업로드

**4. 자동 비행 (Fly View)**
- ARM → Takeoff
- "Start Mission" 버튼
- 실시간 경로 추적, 텔레메트리 확인

**5. 로그 다운로드 (Analyze View)**
- Settings → Analyze → Download Logs
- Flight Review (logs.px4.io)에서 분석

---

## ⭐ 슬라이드 58: GCS 실습 - Mission Planner + MAVLink

**시나리오:** Python으로 Mission Planner 없이 MAVLink 직접 제어

**1. PyMAVLink 설치**
```bash
pip install pymavlink
```

**2. 기본 연결 및 ARM**
```python
from pymavlink import mavutil

# SITL 연결 (UDP)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (master.target_system, master.target_component))

# ARM
master.arducopter_arm()
master.motors_armed_wait()
print("Armed!")

# Takeoff (10m)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10)  # 10m altitude
```

**3. 미션 업로드**
```python
# Waypoint 생성
waypoints = [
    (35.123, 126.456, 10),   # lat, lon, alt
    (35.124, 126.457, 15),
    (35.125, 126.458, 10),
]

# Mission Item 전송
for seq, (lat, lon, alt) in enumerate(waypoints):
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0,
        lat, lon, alt)
```

---

## ⭐ 슬라이드 59: MAVLink 프로토콜 구조

**MAVLink (Micro Air Vehicle Link)**

* **목적:** 경량, 효율적인 드론-GCS 통신
* **버전:** MAVLink 1.0, **MAVLink 2.0** (서명 지원, 확장성)
* **전송:** UART, USB, UDP, TCP

**패킷 구조 (MAVLink 2.0):**
```
[STX][LEN][INC][CMP][SEQ][SYS][COM][MSG_ID(24bit)][PAYLOAD][CRC][SIGNATURE(optional)]
```

**주요 메시지 타입:**

| MSG_ID | 메시지명 | 설명 |
|--------|---------|------|
| 0 | HEARTBEAT | 시스템 상태 (1Hz) |
| 30 | ATTITUDE | 자세 (Roll, Pitch, Yaw) |
| 33 | GLOBAL_POSITION_INT | GPS 위치 |
| 74 | VFR_HUD | HUD 데이터 (속도, 고도) |
| 76 | COMMAND_LONG | 명령 전송 (ARM, Takeoff 등) |
| 147 | BATTERY_STATUS | 배터리 상태 |

**보안 (MAVLink 2):**
- 메시지 서명 (Signature): 해커의 가짜 명령 차단
- 암호화 키 사전 공유 (Pre-shared Secret Key)

---

## ⭐ 슬라이드 60: GCS 통신 아키텍처 (Blueye-1K 예시)

**프로젝트 자료 기반 구조:**

```
┌─────────────────────────────────────────────────────────┐
│                   GCS (Mission Planner)                 │
│  • 임무 계획                                              │
│  • 파라미터 설정                                           │
│  • 실시간 모니터링                                         │
│  • 영상 수신 (RTSP)                                       │
└────────────────────┬────────────────────────────────────┘
                     │ MAVLink (UDP/TCP)
              ┌──────┴──────┐
              │ Telemetry   │ (2.4GHz / 5.8GHz)
              │ (LTE Modem) │ 또는 LTE
              └──────┬──────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│           Main Controller (영상처리보드)                  │
│  • MAVLink Protocol 처리                                │
│  • Mission Planner Porting                             │
│  • RTSP Server (영상 스트리밍)                           │
└────────────────────┬────────────────────────────────────┘
                     │ MAVLink (UART/I2C)
┌────────────────────┴────────────────────────────────────┐
│         Flight Controller (Blueye-FC / Pixhawk)        │
│  • 비행 제어 (PX4/ArduPilot)                            │
│  • 센서 융합 (IMU, GPS, Barometer)                      │
│  • 모터 제어 (PWM → ESC)                                │
└─────────────────────────────────────────────────────────┘
```

**통신 경로:**
1. **GCS ↔ Telemetry:** 무선 (2.4GHz, 5.8GHz, LTE)
2. **Telemetry ↔ Main Controller:** USB/UART
3. **Main Controller ↔ Flight Controller:** UART (57600 baud)

**주요 기능:**
- GCS에서 MAVLink 명령 전송 → Main Controller → FC
- FC 텔레메트리 → Main Controller → GCS
- 카메라 영상 → Main Controller (RTSP Server) → GCS

---

## ⭐ 슬라이드 61: GCS 고급 활용 - 다중 기체 관제

**시나리오:** 여러 드론을 하나의 GCS로 동시 제어 (UAM 관제 시뮬레이션)

**1. MAVLink System ID 설정**
- 각 기체에 고유한 System ID 부여 (1, 2, 3, ...)
- PX4: `MAV_SYS_ID` 파라미터
- ArduPilot: `SYSID_THISMAV` 파라미터

**2. QGC 다중 기체 연결**
```bash
# SITL 1 (System ID=1, UDP 14540)
make px4_sitl gazebo

# SITL 2 (System ID=2, UDP 14541)
ARGS="--instance 1" make px4_sitl gazebo
```

QGC에서 두 기체 모두 자동 인식 → 개별 제어 가능

**3. MAVProxy 네트워킹**
```bash
# MAVProxy를 라우터로 사용
mavproxy.py --master=udp:14540 --out=udp:192.168.1.10:14550 --out=udp:192.168.1.11:14550

# 여러 GCS가 하나의 기체에 동시 연결 가능
```

**활용 사례:**
- **UAM 관제:** 수십 대의 드론을 중앙 GCS에서 모니터링
- **드론 쇼:** 동기화된 비행 패턴 (Intel Shooting Star)
- **수색 구조:** 여러 드론으로 지역 분할 탐색

---

## ⭐ 슬라이드 62: GCS + ROS 2 통합

**"GCS는 사람용, ROS 2는 AI/자율 시스템용"**

**아키텍처:**
```
┌──────────────────┐       ┌──────────────────┐
│  QGroundControl  │◄─────►│  PX4/ArduPilot   │
│  (사람 조종사)     │MAVLink│  (Flight Control)│
└──────────────────┘       └────────┬─────────┘
                                    │
                                    │ MicroXRCE-DDS
                                    ▼
                           ┌──────────────────┐
                           │   ROS 2 노드      │
                           │  (자율 임무 AI)   │
                           │  • SLAM          │
                           │  • 장애물 회피    │
                           │  • 경로 계획      │
                           └──────────────────┘
```

**시나리오:**
1. **정상:** AI가 ROS 2로 자율 비행
2. **비상:** 조종사가 GCS로 수동 개입 (RC Override)

**실습 (PX4 + ROS 2):**
```bash
# PX4 측
make px4_sitl_rtps gazebo

# ROS 2 측
ros2 run px4_ros_com sensor_combined_listener
# /fmu/out/sensor_combined 토픽 구독 → IMU 데이터 확인
```

---

## ⭐ 슬라이드 63: GCS 보안 및 인증

**위협:**
1. **GPS 스푸핑:** 가짜 GPS 신호 주입
2. **MAVLink 가로채기:** 중간자 공격 (Man-in-the-Middle)
3. **RC Override:** 비인가 조종기 제어

**방어 (SW 개발 관점):**

**1. MAVLink 2 서명 (Signing)**
```python
# GCS 측
from pymavlink import mavutil

mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# 서명 활성화
mav.setup_signing(secret_key=b"my_secret_key_12345678901234567890", 
                  sign_outgoing=True, 
                  allow_unsigned_callback=None)
```

FCU 파라미터:
```
MAV_2_SIGNING=1        # 서명 활성화
MAV_2_SECRET_KEY=...   # 32바이트 비밀 키
```

**2. 암호화 (TLS over MAVLink)**
- MAVLink 자체는 평문, 추가 암호화 레이어 필요
- VPN/SSH 터널링 (LTE 모뎀 사용 시)

**3. GPS 교차 검증**
```cpp
// PX4 EKF2 코드 예시
if (gps_velocity_change > imu_acceleration * dt * threshold) {
    // GPS 스푸핑 의심 → GPS 신뢰도 낮춤
    _gps_check_fail_status.flags.spoofed = true;
}
```

---

## ⭐ 슬라이드 64: Week 4 요약 - GCS 핵심

**학습 내용:**
- ✅ GCS 역할: 임무 계획, 모니터링, 펌웨어 관리, 로그 분석
- ✅ 주요 GCS 비교: QGroundControl (크로스), Mission Planner (ArduPilot)
- ✅ MAVLink 프로토콜: 메시지 구조, 보안 (서명)
- ✅ 실습: SITL + QGC/Mission Planner, PyMAVLink
- ✅ 고급: 다중 기체 관제, ROS 2 통합, 보안

**다음 학습 (선택적 Week 5):**
- 펌웨어 내부 구조 심화
- 커스텀 MAVLink 메시지 정의
- 통합 프로젝트: 자율 배송 드론 시스템

**추가 자료:**
- MAVLink Developer Guide: https://mavlink.io/en/
- QGC User Guide: https://docs.qgroundcontrol.com
- Mission Planner Docs: https://ardupilot.org/planner/

---

**END OF WEEK 3 LECTURE PLAN (V2) + WEEK 4 GCS PREVIEW**

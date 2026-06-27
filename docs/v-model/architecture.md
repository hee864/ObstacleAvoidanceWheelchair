# Architecture and Detailed Design

## 컴포넌트

| ID | 컴포넌트 | 책임 | 구현 위치 |
| --- | --- | --- | --- |
| WC-CMP-001 | Joystick Acquisition | X/Y ADC 취득과 이동평균필터 | `Core/Src/main.c` |
| WC-CMP-002 | Command Interpreter | 데드존 및 진행 방향 판정 | `Core/Src/main.c` |
| WC-CMP-003 | Safety Interlock | 전·후방 장애물에 따른 방향별 차단 | `Core/Src/main.c` |
| WC-CMP-004 | Motor Output | 좌우 모터 방향 GPIO와 PWM 출력 | `Core/Src/main.c`, `TryMotorJoystick.ioc` |
| WC-CMP-005 | Perception | YOLOv8 장애물 탐지 | Raspberry Pi 모듈 |
| WC-CMP-006 | Attitude Control | MPU6050 취득, 필터링, 수평 보정 | Raspberry Pi/STM32 자세 모듈 |
| WC-CMP-007 | Diagnostics | 센서 및 제어 상태 출력 | USART3 |

## 인터페이스

| ID | 송신자 → 수신자 | 데이터 | 제약 |
| --- | --- | --- | --- |
| WC-IF-001 | Joystick → STM32 | ADC IN10/IN11 | 12-bit right-aligned ADC |
| WC-IF-002 | Raspberry Pi → STM32 | Front/Rear obstacle | PB10/PB4, pull-down |
| WC-IF-003 | STM32 → Motor Driver | Direction/PWM | PA5, PA6, PA9, PC7, TIM2/3 CH2 |
| WC-IF-004 | MPU6050 → Raspberry Pi | Acceleration/Gyro | I2C |
| WC-IF-005 | Raspberry Pi → STM32 | Filtered attitude | UART |
| WC-IF-006 | STM32 → Debug Console | State and sensor values | USART3, 115200 baud |

## 이동평균필터 설계

`MovingAverageFilter_t`는 8개 샘플 배열, 누적합, 쓰기 인덱스와 유효 샘플 수를 가진다. 새 샘플 입력 시 가장 오래된 값을 누적합에서 제거하고 새 값을 더한다. 계산 복잡도는 샘플당 O(1)이며 X/Y축은 독립 상태를 사용한다. 구현은 `Core/Src/moving_average_filter.c`, 호스트 단위 시험은 `tests/test_moving_average_filter.c`에 위치한다.

## 안전 상태

안전 상태는 좌우 모터 방향 출력 PA5, PA6, PA9, PC7을 LOW로 설정한 상태이다. 다음 조건에서 안전 상태를 적용한다.

- 조이스틱 제어 비활성
- 진행 방향과 일치하는 장애물 입력 활성
- ADC 취득 실패 또는 유효 명령 부재

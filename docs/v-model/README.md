# V-Model Development Process

## 적용 범위

조이스틱 입력, 장애물 감지 연동, 좌우 DC 모터 제어, 자세 보정 인터페이스를 포함하는 휠체어 전동 보조 모듈을 대상으로 한다. STM32 펌웨어와 Raspberry Pi 인식 모듈의 책임을 분리하고 요구사항부터 검증 근거까지 양방향으로 연결한다.

이 문서는 Automotive SPICE 4.0의 SYS/SWE 프로세스와 SUP.8, SUP.9, SUP.10의 산출물 구조, ISO 26262-8의 지원 프로세스 개념을 참고한다. 규격 인증 또는 공식 준수를 의미하지 않는다.

## V-모델 매핑

| 개발 단계 | 산출물 | 대응 검증 단계 | 검증 근거 |
| --- | --- | --- | --- |
| 사용자 요구사항 | [requirements.md](requirements.md) STK 항목 | 시스템 검증 | 완성 시스템, 주행·정지 동작 |
| 시스템 요구사항 | [requirements.md](requirements.md) SYS 항목 | 시스템 통합 시험 | STM32, Raspberry Pi, 모터 인터페이스 시험 |
| 시스템 아키텍처 | [architecture.md](architecture.md) | 인터페이스 시험 | ADC, GPIO, PWM, UART 신호 확인 |
| 소프트웨어 요구사항 | [requirements.md](requirements.md) SWR 항목 | 소프트웨어 시험 | 경곗값 및 안전 상태 시험 |
| 상세 설계와 구현 | `Core/Src/main.c` | 단위 검증 | 필터, 데드존, 인터록 시험 |

## 시스템 경계

```text
Joystick --ADC--> STM32 Control --GPIO/PWM--> Left/Right Motor
                         ^
                         |
Webcam --> Raspberry Pi Perception --GPIO Front/Rear Obstacle

MPU6050 --I2C--> Raspberry Pi Attitude --UART--> STM32 --> Linear Actuator
```

STM32 저장소는 조이스틱과 장애물 신호를 이용한 구동 제어를 포함한다. YOLOv8 추론, MPU6050 취득·필터링과 자세 계산은 Raspberry Pi 모듈에 할당한다.

## 산출물

- [요구사항 명세](requirements.md)
- [시스템 및 소프트웨어 아키텍처](architecture.md)
- [검증 및 확인 명세](verification-validation.md)
- [요구사항 추적성 매트릭스](traceability-matrix.md)
- [지원 프로세스](supporting-processes.md)


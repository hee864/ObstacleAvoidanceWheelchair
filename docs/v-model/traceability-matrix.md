# Bidirectional Traceability Matrix

| Stakeholder | System | Software | Architecture/Code | Verification |
| --- | --- | --- | --- | --- |
| WC-STK-001 | WC-SYS-001, WC-SYS-002, WC-SYS-006 | WC-SWR-001, WC-SWR-002 | WC-CMP-001/002/004, `moving_average_filter.c`, `main.c` | `tests/test_moving_average_filter.c`, WC-UT-001~004, WC-ST-001 |
| WC-STK-002 | WC-SYS-003, WC-SYS-004 | WC-SWR-003, WC-SWR-005 | WC-CMP-003, PB10/PB4 처리 | WC-IT-001~004, WC-ST-002/003 |
| WC-STK-003 | WC-SYS-005 | WC-SWR-004 | PA0 토글 및 안전 상태 분기 | WC-IT-005, WC-ST-005 |
| WC-STK-004 | WC-SYS-007 | Raspberry Pi 자세 필터 요구사항 | WC-CMP-006, WC-IF-004/005 | WC-ST-004 |
| WC-STK-001 | WC-SYS-008 | 진단 출력 요구사항 | WC-CMP-007, USART3 | UART 로그 확인 |

## 변경 영향 확인

요구사항, 인터페이스, 핀맵, 필터 윈도우 또는 데드존이 변경되면 같은 행의 코드와 시험 항목을 함께 검토한다. 추적 링크의 존재뿐 아니라 요구사항과 시험 판정 기준의 의미가 일치하는지 확인한다.

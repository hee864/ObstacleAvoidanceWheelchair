# Host Unit Tests

이 테스트는 STM32 HAL과 독립된 이동평균필터를 호스트 C 컴파일러에서 검증한다.

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -ICore/Inc \
  Core/Src/moving_average_filter.c \
  tests/test_moving_average_filter.c \
  -o test_moving_average_filter

./test_moving_average_filter
```

검증 항목:

- `WC-UT-001`: 초기 샘플 구간의 평균
- `WC-UT-002`: 8-sample sliding window
- X/Y축 필터 상태의 독립성

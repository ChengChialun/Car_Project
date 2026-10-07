#include <micro_seconds_control.h>
static uint32_t us_factor;

// DWT：Data Watch point and Trace
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable Trace
    DWT->CYCCNT = 0;                                // reset CPU cycle Count
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // enable DWT cycle count

    us_factor = SystemCoreClock / 1000000;
}

void Delay_us(uint32_t us) {
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = us * us_factor;
    while ((DWT->CYCCNT - start_tick) < delay_ticks);
}

uint32_t Get_Us_Timestamp(void) {
    return (DWT->CYCCNT / us_factor);
}

// start time
uint32_t DWT_GetTick(void) {
    return DWT->CYCCNT;
}

// end-start time = duration
uint32_t DWT_ElapsedUs(uint32_t start) {
    return (DWT->CYCCNT - start) / us_factor;
}

uint32_t DWT_Now_Us(uint32_t elapsed_us) {
    return DWT->CYCCNT / us_factor + elapsed_us;
}

// for log used
uint64_t Get_SystemTime_Us(void) {
    static uint32_t last = 0;
    static uint64_t high = 0;

    uint32_t now = DWT->CYCCNT;

    if (now < last) {
        // overflow
        high += (uint64_t)1 << 32;
    }

    last = now;

    return ((high | now) / us_factor);
}

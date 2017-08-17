#include "pressure.h"
#include "Repetier.h"

#define AVG_SIZE    16

static uint_fast8_t current = 0;
static bool avgReady = false;
static uint16_t buff[AVG_SIZE];

#define MAX_FORCE   100.0   // Newtons
#define MIN_RAW     620u    // 0.5V/3.3V @ 12bit
#define MAX_RAW     1862u   // 1.5V/3.3V @ 12bit 

static inline float pressureRawToN(uint16_t raw)
{
    int16_t tmp;
    /* 12Bit adc / 3.3 Ref*/
    tmp = raw;
    /* Remove offset*/
    tmp -= MIN_RAW;
    /* Return pressure in newtons*/
    return MAX_FORCE/((float)(MAX_RAW - MIN_RAW))*tmp;
}

/* Called from interrupt ~100ms*/
void pressurePush(uint16_t raw)
{
    if (current >= AVG_SIZE) current = 0;
    /* If this thing ends up needing calibration this is where it should go*/
    buff[current++] = raw;
    if (!avgReady && current == AVG_SIZE) avgReady = true;
}

float pressureGetAvg(void)
{
    uint32_t sum = 0;
    if (!avgReady && !current) return 0; // Not a single sample yet
    for (uint8_t i = 0; i < (avgReady ? AVG_SIZE : current); i++) {
        sum += buff[i];
    }
    return pressureRawToN(avgReady ? sum / AVG_SIZE : sum / current);
}

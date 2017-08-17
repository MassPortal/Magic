#ifndef PRESSURE_H
#define PRESSURE_H

#include <stdint.h>
#include <stdbool.h>

void pressurePush(uint16_t raw);
float pressureGetAvg(void);

#endif /*PRESSURE_H*/

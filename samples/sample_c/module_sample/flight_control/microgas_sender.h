#ifndef MICROGAS_SENDER_H
#define MICROGAS_SENDER_H
#include <stdint.h>
#include "dji_typedef.h"
/* Call DjiLowSpeedDataChannel_Init once after DjiCore_Init, not per sample.
 * boot_id must change on sender restart; sequence increments per NEW sensor sample.
 * uptime_ms is acquisition time since sender boot (not UTC).
 */
T_DjiReturnCode MicroGas_SendCo2(uint32_t boot_id, uint32_t sequence,
                               uint64_t uptime_ms, float co2_ppm);
/* V2 is serialized explicitly, never send this struct with sizeof(struct). */
typedef struct {
    float co2_ppm, temperature_c, humidity_pct;
    float pm1_0, pm2_5, pm4_0, pm10, voc_index, nox_index;
} MicroGasMeasurement;
T_DjiReturnCode MicroGas_SendMeasurement(uint32_t boot_id, uint32_t sequence,
                                        uint64_t uptime_ms, const MicroGasMeasurement *sample);
#endif

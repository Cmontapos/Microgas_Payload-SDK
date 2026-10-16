#ifndef MICROGAS_SENDER_H
#define MICROGAS_SENDER_H
#include <stddef.h>
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

/* V3 = v2 bytes 0-59 + latitude/longitude (float64, degrees) + WGS84 altitude (float32)
 * + CRC32 of bytes 0-79. Shown as one text line in the Pilot 2 floating window, where
 * MicroGas reads it without MSDK: "MG3:" + base64 (112 characters, no padding).
 * Pass NAN position when there is no valid fix. Returns line length, or -1 if invalid.
 */
#define MICROGAS_V3_PACKET_SIZE 84
#define MICROGAS_WINDOW_LINE_PREFIX "MG3:"
#define MICROGAS_WINDOW_LINE_MAX 128
int MicroGas_EncodeWindowLine(char *out, size_t size, uint32_t boot_id, uint32_t sequence,
                              uint64_t uptime_ms, const MicroGasMeasurement *sample,
                              double latitude_deg, double longitude_deg, float altitude_m);
#endif

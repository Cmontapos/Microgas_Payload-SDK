#include "microgas_sender.h"
#include "dji_low_speed_data_channel.h"
#include <math.h>
#include <string.h>

static void put32(uint8_t *out, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

T_DjiReturnCode MicroGas_SendCo2(uint32_t boot_id, uint32_t sequence,
                               uint64_t uptime_ms, float co2_ppm) {
    _Static_assert(sizeof(float) == 4, "Protocol requires 32-bit IEEE-754 float");
    if (!isfinite(co2_ppm) || co2_ppm < 0 || uptime_ms > INT64_MAX)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    uint8_t packet[32] = {'M', 'G', 'A', 'S', 1, 1, 0, 0};
    put32(packet + 8, boot_id);
    put32(packet + 12, sequence);
    for (unsigned i = 0; i < 8; ++i) packet[16 + i] = (uint8_t)(uptime_ms >> (8 * i));
    uint32_t bits;
    memcpy(&bits, &co2_ppm, 4);
    put32(packet + 24, bits);
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 28; ++i) {
        crc ^= packet[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
    }
    put32(packet + 28, crc ^ UINT32_MAX);
    return DjiLowSpeedDataChannel_SendData(DJI_CHANNEL_ADDRESS_MASTER_RC_APP,
                                         packet, (uint8_t)sizeof(packet));
}


T_DjiReturnCode MicroGas_SendMeasurement(uint32_t boot_id, uint32_t sequence,
                                        uint64_t uptime_ms, const MicroGasMeasurement *sample) {
    _Static_assert(sizeof(float) == 4, "Protocol requires 32-bit IEEE-754 float");
    if (!sample || !isfinite(sample->co2_ppm) || sample->co2_ppm < 0 || uptime_ms > INT64_MAX)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    uint8_t packet[64] = {'M', 'G', 'A', 'S', 2, 1, 0, 0};
    put32(packet + 8, boot_id);
    put32(packet + 12, sequence);
    for (unsigned i = 0; i < 8; ++i) packet[16 + i] = (uint8_t)(uptime_ms >> (8 * i));
    const float values[9] = {sample->co2_ppm, sample->temperature_c, sample->humidity_pct,
        sample->pm1_0, sample->pm2_5, sample->pm4_0, sample->pm10, sample->voc_index, sample->nox_index};
    for (unsigned i = 0; i < 9; ++i) {
        uint32_t bits;
        float value = values[i];
        if (!isfinite(value) || (i == 2 && (value < 0 || value > 100)) ||
            (i >= 3 && i <= 6 && value < 0) || (i >= 7 && (value < 1 || value > 500)))
            value = NAN;
        memcpy(&bits, &value, 4);
        put32(packet + 24 + 4 * i, bits);
    }
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 60; ++i) {
        crc ^= packet[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
    }
    put32(packet + 60, crc ^ UINT32_MAX);
    return DjiLowSpeedDataChannel_SendData(DJI_CHANNEL_ADDRESS_MASTER_RC_APP, packet, (uint8_t)sizeof(packet));
}

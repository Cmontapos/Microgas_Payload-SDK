#include "microgas_sender.h"
#include "dji_low_speed_data_channel.h"
#include <math.h>
#include <string.h>

static void put32(uint8_t *out, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

static void put64(uint8_t *out, uint64_t value) {
    for (unsigned i = 0; i < 8; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

static void putFloat(uint8_t *out, float value) {
    uint32_t bits;
    memcpy(&bits, &value, 4);
    put32(out, bits);
}

static void putDouble(uint8_t *out, double value) {
    uint64_t bits;
    memcpy(&bits, &value, 8);
    put64(out, bits);
}

static uint32_t crc32(const uint8_t *data, unsigned len) {
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < len; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
    }
    return crc ^ UINT32_MAX;
}

/* Header and nine channels shared by v2 and v3 (bytes 0-59). */
static int putMeasurement(uint8_t *packet, uint8_t version, uint32_t boot_id, uint32_t sequence,
                          uint64_t uptime_ms, const MicroGasMeasurement *sample) {
    if (!sample || !isfinite(sample->co2_ppm) || sample->co2_ppm < 0 || uptime_ms > INT64_MAX)
        return -1;
    const uint8_t header[8] = {'M', 'G', 'A', 'S', version, 1, 0, 0};
    memcpy(packet, header, sizeof(header));
    put32(packet + 8, boot_id);
    put32(packet + 12, sequence);
    put64(packet + 16, uptime_ms);
    const float values[9] = {sample->co2_ppm, sample->temperature_c, sample->humidity_pct,
        sample->pm1_0, sample->pm2_5, sample->pm4_0, sample->pm10, sample->voc_index, sample->nox_index};
    for (unsigned i = 0; i < 9; ++i) {
        float value = values[i];
        if (!isfinite(value) || (i == 2 && (value < 0 || value > 100)) ||
            (i >= 3 && i <= 6 && value < 0) || (i >= 7 && (value < 1 || value > 500)))
            value = NAN;
        putFloat(packet + 24 + 4 * i, value);
    }
    return 0;
}

T_DjiReturnCode MicroGas_SendCo2(uint32_t boot_id, uint32_t sequence,
                               uint64_t uptime_ms, float co2_ppm) {
    _Static_assert(sizeof(float) == 4, "Protocol requires 32-bit IEEE-754 float");
    if (!isfinite(co2_ppm) || co2_ppm < 0 || uptime_ms > INT64_MAX)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    uint8_t packet[32] = {'M', 'G', 'A', 'S', 1, 1, 0, 0};
    put32(packet + 8, boot_id);
    put32(packet + 12, sequence);
    put64(packet + 16, uptime_ms);
    putFloat(packet + 24, co2_ppm);
    put32(packet + 28, crc32(packet, 28));
    return DjiLowSpeedDataChannel_SendData(DJI_CHANNEL_ADDRESS_MASTER_RC_APP,
                                         packet, (uint8_t)sizeof(packet));
}


T_DjiReturnCode MicroGas_SendMeasurement(uint32_t boot_id, uint32_t sequence,
                                        uint64_t uptime_ms, const MicroGasMeasurement *sample) {
    _Static_assert(sizeof(float) == 4, "Protocol requires 32-bit IEEE-754 float");
    uint8_t packet[64];
    if (putMeasurement(packet, 2, boot_id, sequence, uptime_ms, sample) != 0)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    put32(packet + 60, crc32(packet, 60));
    return DjiLowSpeedDataChannel_SendData(DJI_CHANNEL_ADDRESS_MASTER_RC_APP, packet, (uint8_t)sizeof(packet));
}

int MicroGas_EncodeWindowLine(char *out, size_t size, uint32_t boot_id, uint32_t sequence,
                              uint64_t uptime_ms, const MicroGasMeasurement *sample,
                              double latitude_deg, double longitude_deg, float altitude_m) {
    _Static_assert(sizeof(double) == 8, "Protocol requires 64-bit IEEE-754 double");
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    uint8_t packet[MICROGAS_V3_PACKET_SIZE];
    if (!out || size < MICROGAS_WINDOW_LINE_MAX ||
        putMeasurement(packet, 3, boot_id, sequence, uptime_ms, sample) != 0)
        return -1;
    /* Without a valid fix, all three position fields are NaN (never a partial position). */
    if (!isfinite(latitude_deg) || !isfinite(longitude_deg) ||
        fabs(latitude_deg) > 90.0 || fabs(longitude_deg) > 180.0) {
        latitude_deg = NAN;
        longitude_deg = NAN;
        altitude_m = NAN;
    } else if (!isfinite(altitude_m)) {
        altitude_m = NAN;
    }
    putDouble(packet + 60, latitude_deg);
    putDouble(packet + 68, longitude_deg);
    putFloat(packet + 76, altitude_m);
    put32(packet + 80, crc32(packet, 80));

    /* 84 bytes = 28 groups of 3, so base64 needs no padding. */
    size_t n = 0;
    memcpy(out, MICROGAS_WINDOW_LINE_PREFIX, sizeof(MICROGAS_WINDOW_LINE_PREFIX) - 1);
    n += sizeof(MICROGAS_WINDOW_LINE_PREFIX) - 1;
    for (unsigned i = 0; i < sizeof(packet); i += 3) {
        uint32_t group = (uint32_t)packet[i] << 16 | (uint32_t)packet[i + 1] << 8 | packet[i + 2];
        for (int shift = 18; shift >= 0; shift -= 6) out[n++] = alphabet[(group >> shift) & 0x3F];
    }
    out[n] = '\0';
    return (int)n;
}

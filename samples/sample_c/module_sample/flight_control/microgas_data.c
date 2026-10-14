#include "microgas_data.h"
#include "dji_logger.h"
#include "dji_platform.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#define SEN66_I2C_BUS      "/dev/i2c-1"
#define SEN66_I2C_ADDRESS  0x6B

#define MICROGAS_DIR       "/home/minihawk/MiniHawkData"
#define MICROGAS_CSV       "/home/minihawk/MiniHawkData/mediciones.csv"

static int sen66_fd = -1;


/* =========================================================
 * CRC Sensirion
 * polynomial = 0x31
 * initial value = 0xFF
 * ========================================================= */
static uint8_t MicroGas_CalcCrc(const uint8_t *data, uint8_t count)
{
    uint8_t crc = 0xFF;

    for (uint8_t i = 0; i < count; i++) {

        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {

            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


/* =========================================================
 * Enviar comando de 16 bits al SEN66
 * ========================================================= */
static int MicroGas_SendCommand(uint16_t command)
{
    uint8_t buffer[2];

    buffer[0] = (command >> 8) & 0xFF;
    buffer[1] = command & 0xFF;

    if (write(sen66_fd, buffer, 2) != 2) {
        USER_LOG_WARN("SEN66: escritura I2C incompleta o fallida: %s", strerror(errno));
        return -1;
    }

    return 0;
}


/* =========================================================
 * Inicializar SEN66
 * ========================================================= */
int MicroGas_Sen66Init(void)
{
    if (sen66_fd >= 0) {
        return 0;
    }

    sen66_fd = open(SEN66_I2C_BUS, O_RDWR | O_CLOEXEC);

    if (sen66_fd < 0) {
        USER_LOG_ERROR("SEN66: no se pudo abrir %s: %s", SEN66_I2C_BUS, strerror(errno));
        return -1;
    }

    if (ioctl(sen66_fd, I2C_SLAVE, SEN66_I2C_ADDRESS) < 0) {
        USER_LOG_ERROR("SEN66: ioctl I2C_SLAVE: %s", strerror(errno));
        close(sen66_fd);
        sen66_fd = -1;
        return -1;
    }

    /* Volver a idle una vez: el sensor puede seguir midiendo tras reiniciar PSDK. */
    if (MicroGas_SendCommand(0x0104) != 0) {
        close(sen66_fd);
        sen66_fd = -1;
        return -1;
    }
    DjiPlatform_GetOsalHandler()->TaskSleepMs(1400);

    /*
     * Start Continuous Measurement
     * Command ID = 0x0021
     */
    if (MicroGas_SendCommand(0x0021) != 0) {
        close(sen66_fd);
        sen66_fd = -1;
        return -1;
    }

    /*
     * El SEN66 necesita ~1.1 s para la primera medición.
     * Esperamos un poco más para estar seguros.
     */
    DjiPlatform_GetOsalHandler()->TaskSleepMs(2000);

    USER_LOG_INFO("SEN66 inicializado correctamente en %s, 0x6B", SEN66_I2C_BUS);

    return 0;
}


/* =========================================================
 * Leer word uint16 con CRC
 * ========================================================= */
static int MicroGas_GetWord(const uint8_t *buffer,
                            int offset,
                            uint16_t *value)
{
    uint8_t data[2];

    data[0] = buffer[offset];
    data[1] = buffer[offset + 1];

    uint8_t receivedCrc = buffer[offset + 2];

    if (MicroGas_CalcCrc(data, 2) != receivedCrc) {

        USER_LOG_WARN("SEN66: CRC incorrecto en byte %d", offset);
        return -1;
    }

    *value =
        ((uint16_t)data[0] << 8) |
         (uint16_t)data[1];

    return 0;
}


/* =========================================================
 * Leer las 9 variables del SEN66
 * ========================================================= */
int MicroGas_ReadSen66(MicroGasData *data)
{
    uint8_t buffer[27];

    uint16_t rawPm1;
    uint16_t rawPm25;
    uint16_t rawPm4;
    uint16_t rawPm10;

    uint16_t rawHumidityU;
    uint16_t rawTemperatureU;
    uint16_t rawVocU;
    uint16_t rawNoxU;

    uint16_t rawCo2;
    uint16_t ready;

    if (data == NULL || sen66_fd < 0) {
        return -1;
    }

    /* Get Data Ready: no repetir una muestra anterior del sensor. */
    if (MicroGas_SendCommand(0x0202) != 0) {
        return -1;
    }
    DjiPlatform_GetOsalHandler()->TaskSleepMs(20);
    if (read(sen66_fd, buffer, 3) != 3) {
        USER_LOG_WARN("SEN66: no se pudo leer Data Ready");
        return -1;
    }
    if (MicroGas_GetWord(buffer, 0, &ready) != 0) {
        return -1;
    }
    if ((ready & 0xFF) == 0) {
        return 1;
    }

    /*
     * Read Measured Values
     * Command = 0x0300
     */
    if (MicroGas_SendCommand(0x0300) != 0) {
        return -1;
    }

    /*
     * El comando necesita aprox. 20 ms.
     */
    DjiPlatform_GetOsalHandler()->TaskSleepMs(20);

    if (read(sen66_fd, buffer, sizeof(buffer)) != sizeof(buffer)) {

        USER_LOG_WARN("SEN66: lectura I2C incompleta o fallida: %s", strerror(errno));
        return -1;
    }

    /*
     * 9 valores × (2 bytes + CRC)
     */

    if (MicroGas_GetWord(buffer, 0,  &rawPm1) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 3,  &rawPm25) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 6,  &rawPm4) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 9,  &rawPm10) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 12, &rawHumidityU) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 15, &rawTemperatureU) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 18, &rawVocU) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 21, &rawNoxU) != 0)
        return -1;

    if (MicroGas_GetWord(buffer, 24, &rawCo2) != 0)
        return -1;


    /* Sentinelas documentados: no guardar datos desconocidos/calentamiento. */
    if (rawPm1 == UINT16_MAX || rawPm25 == UINT16_MAX ||
        rawPm4 == UINT16_MAX || rawPm10 == UINT16_MAX || rawCo2 == UINT16_MAX ||
        rawHumidityU == INT16_MAX || rawTemperatureU == INT16_MAX ||
        rawVocU == INT16_MAX || rawNoxU == INT16_MAX) {
        return 1;
    }

    /*
     * Humedad, temperatura, VOC y NOx son int16_t.
     */
    int16_t rawHumidity =
        (int16_t)rawHumidityU;

    int16_t rawTemperature =
        (int16_t)rawTemperatureU;

    int16_t rawVoc =
        (int16_t)rawVocU;

    int16_t rawNox =
        (int16_t)rawNoxU;


    /*
     * Aplicar escalas documentadas por Sensirion.
     */
    data->pm1_0 =
        rawPm1 / 10.0f;

    data->pm2_5 =
        rawPm25 / 10.0f;

    data->pm4_0 =
        rawPm4 / 10.0f;

    data->pm10 =
        rawPm10 / 10.0f;

    data->humidity_rh =
        rawHumidity / 100.0f;

    data->temperature_c =
        rawTemperature / 200.0f;

    data->voc_index =
        rawVoc / 10.0f;

    data->nox_index =
        rawNox / 10.0f;

    data->co2_ppm =
        rawCo2;


    return 0;
}


/* =========================================================
 * Guardar CSV
 * ========================================================= */
int MicroGas_SaveCsv(const MicroGasData *data)
{
    int result = 0;
    struct stat status;
    FILE *file;

    if (data == NULL) {
        return -1;
    }
    if (mkdir(MICROGAS_DIR, 0755) != 0 && errno != EEXIST) {
        USER_LOG_ERROR("MiniHawk: mkdir %s: %s", MICROGAS_DIR, strerror(errno));
        return -1;
    }
    /* Append; fstat tambien permite recuperar un archivo vacio tras un fallo. */
    file = fopen(MICROGAS_CSV, "a");
    if (file == NULL) {
        USER_LOG_ERROR("MiniHawk: fopen %s: %s", MICROGAS_CSV, strerror(errno));
        return -1;
    }
    if (fstat(fileno(file), &status) != 0) {
        USER_LOG_ERROR("MiniHawk: fstat CSV: %s", strerror(errno));
        fclose(file);
        return -1;
    }

    if (status.st_size == 0) {

        if (fprintf(file,
            "timestamp,"
            "hora,"
            "latitud,"
            "longitud,"
            "altitud_m,"
            "co2_ppm,"
            "temperatura_c,"
            "humedad_rh,"
            "pm1_0_ug_m3,"
            "pm2_5_ug_m3,"
            "pm4_0_ug_m3,"
            "pm10_ug_m3,"
            "voc_index,"
            "nox_index\n") < 0) {
            result = -1;
        }
    }


    if (result == 0 && fprintf(file,

        "%s,"
        "%s,"
        "%.8f,"
        "%.8f,"
        "%.3f,"
        "%u,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f\n",

        data->timestamp,
        data->hora,

        data->latitude_deg,
        data->longitude_deg,
        data->altitude_m,

        (unsigned int)data->co2_ppm,

        data->temperature_c,
        data->humidity_rh,

        data->pm1_0,
        data->pm2_5,
        data->pm4_0,
        data->pm10,

        data->voc_index,
        data->nox_index
    ) < 0) {
        result = -1;
    }

    if (fflush(file) != 0) {
        result = -1;
    }
    if (fclose(file) != 0) {
        result = -1;
    }
    if (result != 0) {
        USER_LOG_ERROR("MiniHawk: error escribiendo/cerrando %s: %s", MICROGAS_CSV, strerror(errno));
    }
    return result;
}

#ifndef MICROGAS_DATA_H
#define MICROGAS_DATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* DJI */
    char timestamp[32];
    char hora[16];

    double latitude_deg;
    double longitude_deg;
    float altitude_m;

    /* SEN66 */
    uint16_t co2_ppm;

    float temperature_c;
    float humidity_rh;

    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10;

    float voc_index;
    float nox_index;

} MicroGasData;

/* Un solo propietario: la tarea de adquisicion. Init es idempotente.
 * Read devuelve 0 con una muestra completa, 1 si aun no esta disponible,
 * y -1 ante un error. No modifica los campos DJI. */
int MicroGas_Sen66Init(void);
int MicroGas_ReadSen66(MicroGasData *data);
int MicroGas_SaveCsv(const MicroGasData *data);

#ifdef __cplusplus
}
#endif

#endif

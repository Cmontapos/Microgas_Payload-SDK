# Contexto Técnico: Microgas Payload-SDK (MiniHawk)

Este documento describe la arquitectura, modificaciones y nuevos módulos implementados en este repositorio con respecto al repositorio base original de DJI (**[dji-sdk/Payload-SDK](https://github.com/dji-sdk/Payload-SDK)**, versión base `release/v3.16.0` - commit `9af0853`).

---

## 1. Propósito General del Proyecto

El objetivo de este desarrollo es transformar una **Raspberry Pi** montada en el puerto de carga útil (*Payload Port*) de un dron DJI (como la serie Matrice) en una estación autónoma de monitoreo ambiental denominada **MicroGas MiniHawk**.

El sistema realiza de manera integrada:
1. **Adquisición continua de gases y partículas** mediante un sensor Sensirion SEN66 (I2C).
2. **Sincronización con la telemetría del dron** (fecha, hora, latitud, longitud y altitud WGS84) mediante las suscripciones PSDK de DJI.
3. **Persistencia local de datos** en un archivo CSV en `/home/minihawk/MiniHawkData/mediciones.csv`.
4. **Telemetría en tiempo real hacia el control remoto (RC Pro)** por el *DJI Low Speed Data Channel* utilizando un protocolo binario propio (`MGAS`).
5. **Monitoreo visual en DJI Pilot 2** mediante un widget de ventana flotante con una barra gráfica de $\text{CO}_2$ (0 a 5000 ppm).
6. **Exportación desatendida a llave USB** mediante reglas `udev` y servicios `systemd` que extraen una copia del CSV al conectar cualquier pendrive sin pausar el muestreo.

---

## 2. Historial de Commits sobre DJI PSDK v3.16.0

Sobre la versión oficial `v3.16.0` se introdujeron los siguientes conjuntos de cambios:

1. **Commit `1732b4b` ("Cambios Microgas MiniHawk"):**
   - Integración completa del sensor Sensirion SEN66 en C.
   - Creación de la tarea principal de adquisición y sincronización GPS/altitud en `test_flight_control.c`.
   - Ajustes de coexistencia de suscripciones FC para evitar errores de tópicos duplicados en DJI PSDK.
   - Implementación del widget en ventana flotante con barra de progreso de $\text{CO}_2$.
   - Suite de herramientas y servicios para exportación automática a USB (`tools/minihawk_usb_export/`).
   - Ajustes de CMake y configuración de hardware para Raspberry Pi (baud rate a 921600 bps).

2. **Commit `36a7a38` ("Cambios para el uso con MDSK en apps de android instaladas en el control RC pro"):**
   - Implementación del protocolo de paquetes binarios `MGAS` (V1 para $\text{CO}_2$ y V2 para todas las variables) con cálculo de CRC32.
   - Transmisión continua a través de `DjiLowSpeedDataChannel_SendData` hacia el canal del control remoto (`DJI_CHANNEL_ADDRESS_MASTER_RC_APP`).

3. **Modificaciones locales de entorno (`hal_uart.h`):**
   - Selección del puerto UART de comunicación con el dron (`/dev/ttyAMA0` o `/dev/ttyAMA1` según el modelo de Pi y cabecera de expansión).

---

## 3. Módulos Desarrollados y Modificados

### A. Módulo de Datos y Sensor SEN66 (`microgas_data`)
* **Archivos:** 
  - `samples/sample_c/module_sample/flight_control/microgas_data.h`
  - `samples/sample_c/module_sample/flight_control/microgas_data.c`

* **Estructura principal (`MicroGasData`):**
  ```c
  typedef struct {
      char timestamp[32];      // ISO 8601 (YYYY-MM-DDTHH:MM:SS)
      char hora[16];           // HH:MM:SS
      double latitude_deg;     // Grados decimales (WGS84)
      double longitude_deg;    // Grados decimales (WGS84)
      float altitude_m;        // Altitud elipsoidal WGS84

      uint16_t co2_ppm;        // Concentración CO2 (ppm)
      float temperature_c;     // Temperatura (°C)
      float humidity_rh;       // Humedad relativa (%)
      float pm1_0;             // PM1.0 (ug/m3)
      float pm2_5;             // PM2.5 (ug/m3)
      float pm4_0;             // PM4.0 (ug/m3)
      float pm10;              // PM10 (ug/m3)
      float voc_index;         // Índice VOC (1..500)
      float nox_index;         // Índice NOx (1..500)
  } MicroGasData;
  ```

* **Funcionalidad clave:**
  - **I2C Linux directo:** Abre `/dev/i2c-1` en dirección `0x6B` con `I2C_SLAVE` e inicializa el sensor enviando el comando `0x0104` (start periodic measurement) tras limpiar estados previos.
  - **Cálculo de CRC Sensirion:** Polinomio $0\text{x}31$ con valor inicial $0\text{xFF}$ para validar cada palabra de 16 bits leída del bus I2C.
  - **Conversión de escalas crudas:**
    - $\text{PM} / 10.0$
    - $\text{Humedad} / 100.0$
    - $\text{Temperatura} / 200.0$
    - $\text{VOC} / 10.0$
    - $\text{NO}_x / 10.0$
  - **Persistencia CSV (`MicroGas_SaveCsv`):**
    - Crea el directorio `/home/minihawk/MiniHawkData` si no existe.
    - Escribe cabeceras en modo `append` solo si el tamaño es 0 bytes.
    - Aplica `fflush()` y `fclose()` de inmediato para mitigar pérdida de datos ante corte de suministro eléctrico.

---

### B. Módulo de Transmisión hacia App Android (`microgas_sender`)
* **Archivos:**
  - `samples/sample_c/module_sample/flight_control/microgas_sender.h`
  - `samples/sample_c/module_sample/flight_control/microgas_sender.c`

* **Protocolo de Enlace `MGAS`:**
  - **Trama V2 (64 bytes):**
    - `Header [0..3]`: `'M', 'G', 'A', 'S'`
    - `Version [4]`: `2` (trama completa de variables)
    - `Flags [5..7]`: `1, 0, 0`
    - `Boot ID [8..11]`: Entero de 32 bits generado por ejecución para diferenciar reinicios.
    - `Sequence [12..15]`: Contador incremental de 32 bits por muestra nueva del sensor.
    - `Uptime ms [16..23]`: Tiempo de ejecución en milisegundos (monotónico).
    - `Payload [24..59]`: 9 valores `float` IEEE 754 de 32 bits (CO2, Temp, Hum, PM1.0, PM2.5, PM4.0, PM10, VOC, NOx).
    - `CRC32 [60..63]`: CRC32 estándar (polinomio reflectado `0xEDB88320`).
  - **Transmisión:** Llama a `DjiLowSpeedDataChannel_SendData(DJI_CHANNEL_ADDRESS_MASTER_RC_APP, packet, 64)`.

---

### C. Núcleo de Adquisición y Vuelo (`test_flight_control`)
* **Archivos:**
  - `samples/sample_c/module_sample/flight_control/test_flight_control.c`
  - `samples/sample_c/module_sample/flight_control/test_flight_control.h`

* **Hilo de Adquisición (`MiniHawk_AcquisitionTask`):**
  - Tarea OSAL con 4 KB de stack (`minihawk_data`).
  - Sincroniza inicio con semáforo `s_miniHawkReady`.
  - **Lectura DJI (`MiniHawk_ReadDjiData`):**
    - Consulta `DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE`, `GPS_TIME` y `POSITION_FUSED`.
    - Realiza validaciones estrictas de rangos calendáricos y número de satélites visibles.
    - Convierte latitud y longitud de radianes a grados y extrae altitud elipsoidal WGS84.
    - **Fallback de contingencia:** Si el dron no tiene fijación GPS o el tópico falla, usa `time()` y `localtime_r()` de Linux para registrar la marca de tiempo local y asigna `NAN` a las coordenadas, garantizando que el sensor siga registrando datos.
  - Guarda atómicamente el último valor de $\text{CO}_2$ en `s_miniHawkCo2Ppm` para consumo del módulo de widgets.

---

### D. Interfaz Visual en DJI Pilot 2 (`test_widget`)
* **Archivo:**
  - `samples/sample_c/module_sample/widget/test_widget.c`

* **Ventana Flotante de $\text{CO}_2$ (`MiniHawk_BuildCo2Window`):**
  - Genera dinámicamente un texto con una barra de progreso fija de 30 caracteres:
    ```
    CO2: 1250 ppm
    [=======-----------------------]
    25.0%
    ```
  - Si no hay lectura válida del sensor, muestra:
    ```
    CO2: -- ppm
    [------------------------------]
    Sin lectura SEN66
    ```
  - Se actualiza cada 1000 ms mediante `DjiWidgetFloatingWindow_ShowMessage`.

---

### E. Concurrencia de Suscripciones FC
* **Problema en DJI PSDK:** Si múltiples módulos llaman a `DjiFcSubscription_Init()` o intentan suscribirse a los mismos tópicos (`VELOCITY`, `GPS_POSITION`, etc.), el SDK retorna error `TOPIC_DUPLICATE` o falla en cascada.
* **Solución introducida:**
  - Se creó la función `MiniHawk_UsesFcSubscription()` en `test_flight_control.c`.
  - En `test_fc_subscription.c`, `test_interest_point.c`, `test_waypoint_v2.c`, `test_gimbal_manager.c` y `test_hms.c` se agregaron comprobaciones para tolerar `TOPIC_DUPLICATE` y reutilizar la suscripción compartida ya iniciada por MiniHawk.

---

### F. Adaptaciones a Raspberry Pi y Sistema Operativo
* **Punto de Entrada (`samples/sample_c++/platform/linux/raspberry_pi/application/main.cpp`):**
  - Llama inmediatamente a `MiniHawk_StartAcquisition()` antes de entrar al menú de comandos interactivos.
* **Alias del Payload (`main.c`):**
  - Configura el alias visible en el sistema DJI como `"MicroGas(DjiHawk"`.
* **Velocidad de Enlace UART (`dji_sdk_app_info.h`):**
  - Cambiada de `460800` a `921600` bps para acomodar la tasa de telemetría y mensajes de widget.
* **Compilación (`CMakeLists.txt`):**
  - Soporte habilitado para C y C++ (`project(dji_sdk_demo_on_rpi_cxx C CXX)`).
  - Define `-DSYSTEM_ARCH_LINUX`.
  - Fuerza la salida en `${CMAKE_CURRENT_SOURCE_DIR}/build/bin` requerida por el servicio *systemd* que ejecuta la aplicación al arrancar la Raspberry Pi.

---

### G. Módulo de Exportación a USB en Caliente (`tools/minihawk_usb_export/`)
* **Archivos:**
  - `tools/minihawk_usb_export/minihawk_usb_export.py`: Script principal de extracción.
  - `tools/minihawk_usb_export/90-minihawk-usb-export.rules`: Regla `udev` para eventos `add` en subsistema `block`.
  - `tools/minihawk_usb_export/minihawk-usb-export@.service`: Servicio instanciado por dispositivo.
  - `tools/minihawk_usb_export/install.sh`: Script de instalación automática en `/etc/udev/rules.d/` y `/etc/systemd/system/`.
  - `tools/minihawk_usb_export/README.md`: Documentación operativa.

* **Mecanismo de Operación:**
  1. Al conectar una memoria USB (FAT32, exFAT, ext4, NTFS), `udev` activa `minihawk-usb-export@<dev>.service`.
  2. El servicio monta el pendrive en un namespace de montaje privado temporal.
  3. Comprueba el archivo `/home/minihawk/MiniHawkData/mediciones.csv`, descarta filas incompletas si el sistema estaba escribiendo en ese instante y copia el contenido a la raíz de la USB como:
     `MicroGas_mediciones_YYYYMMDD_HHMMSS_<identificador>.csv`
  4. Realiza `sync` y desmonta la partición de forma segura.
  5. **Garantía:** No interrumpe la escritura continua del proceso C del PSDK ni bloquea el sistema de archivos principal.

---

## 4. Diagrama de Flujo de Datos

```
                    ┌─────────────────────────┐
                    │    Dron DJI Matrice     │
                    │  (Controladora Vuelo)   │
                    └────────────┬────────────┘
                                 │ UART (921600 bps)
                                 ▼
                     [ PSDK Core / Suscripciones ]
                    (GPS Date/Time, PositionFused)
                                 │
 [ Sensor Sensirion SEN66 ]      │
 (CO2, Temp, Hum, PM, VOC, NOx)  │
           │                     │
           │ I2C (/dev/i2c-1)    │
           ▼                     ▼
     ┌────────────────────────────────────────────────┐
     │           MiniHawk_AcquisitionTask             │
     │      (sample_c/module_sample/flight_control)   │
     └───────┬───────────────────┬────────────────┬───┘
             │                   │                │
             ▼                   ▼                ▼
     [ Guardado CSV ]     [ Sender MGAS ]  [ Widget Pilot 2 ]
     mediciones.csv       Paquete V2 (64B) Floating Window CO2
     (/home/minihawk/...)  LowSpeedChannel  [===------] 35%
             │            -> RC Pro / MSDK
             ▼
    ┌─────────────────┐
    │  udev + systemd │
    │ Exportación USB │
    └─────────────────┘
```

---

## 5. Tabla Resumen de Archivos Principales

| Ruta del Archivo | Estado | Descripción del Cambio |
| :--- | :---: | :--- |
| `samples/sample_c/module_sample/flight_control/microgas_data.h/.c` | **Nuevo** | Driver I2C del sensor Sensirion SEN66 y función `MicroGas_SaveCsv`. |
| `samples/sample_c/module_sample/flight_control/microgas_sender.h/.c` | **Nuevo** | Protocolo binario `MGAS` y transmisión por `LowSpeedDataChannel`. |
| `samples/sample_c/module_sample/flight_control/test_flight_control.h/.c` | **Modificado** | Hilo de adquisición `minihawk_data`, extracción GPS DJI y fallback local. |
| `samples/sample_c/module_sample/widget/test_widget.c` | **Modificado** | Generador de ventana flotante con barra gráfica de $\text{CO}_2$ para DJI Pilot 2. |
| `samples/sample_c/module_sample/fc_subscription/test_fc_subscription.c` | **Modificado** | Prevención de duplicidad de suscripciones FC (`MiniHawk_UsesFcSubscription`). |
| `samples/sample_c++/platform/linux/raspberry_pi/application/main.cpp` | **Modificado** | Arranque automático de `MiniHawk_StartAcquisition()`. |
| `samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_app_info.h` | **Modificado** | Ajuste de baud rate a `921600`. |
| `samples/sample_c/platform/linux/raspberry_pi/hal/hal_uart.h` | **Modificado** | Asignación de puerto UART a `/dev/ttyAMA0`. |
| `samples/sample_c++/platform/linux/raspberry_pi/CMakeLists.txt` | **Modificado** | Flags de compilación y fijación del directorio destino `build/bin`. |
| `tools/minihawk_usb_export/*` | **Nuevo** | Suite de exportación automática a llave USB (`udev` + `systemd`). |


---

## 6. Línea MG3 para la app del control sin MSDK (2026-10-16)

* `microgas_sender.c/.h`: `MicroGas_EncodeWindowLine` serializa el paquete v3 (84 B:
  v2 + latitud/longitud float64 + altitud float32 + CRC32) como `MG3:` + base64.
  v1 y v2 por `LowSpeedDataChannel` siguen iguales (verificado contra el fixture).
* `test_flight_control.c`: tras cada muestra nueva guarda la línea MG3 con boot,
  secuencia, uptime, nueve variables y GPS (NaN sin fix); `MiniHawk_GetWindowLine()`
  la entrega protegida con mutex.
* `test_widget.c`: añade la línea MG3 debajo del texto de CO₂ y refresca cada 500 ms
  (~340 B/s, límite 2 KB/s). La app `MicroGas Pilot` del RC Pro la lee desde Pilot 2.

# Exportacion del CSV a USB

Al conectar una llave USB, udev inicia un servicio systemd que monta la particion
en un espacio privado, copia una instantanea del CSV y desmonta ese montaje.
La adquisicion sigue escribiendo el archivo original sin bloquearse.

Origen: `/home/minihawk/MiniHawkData/mediciones.csv`.
Destino: raiz de la llave, `MicroGas_mediciones_FECHA_HORA_IDENTIFICADOR.csv`.
La fecha del nombre usa el reloj UTC de la Pi; el identificador evita colisiones
aunque el reloj no este ajustado. Se omite la ultima fila si esta incompleta.
Cada conexion genera una copia nueva. No borra ni modifica el CSV original.

La llave debe tener un sistema de archivos ya creado y espacio disponible;
FAT32 y exFAT son las opciones habituales. Tambien se admiten NTFS y ext2/3/4
si el sistema de la Pi permite montarlos. No se formatea ningun dispositivo.
Se excluyen las particiones usadas por el sistema y por el CSV de origen.
Si aun no hay mediciones completas, hay que conectar la llave nuevamente
despues de que MiniHawk haya guardado datos. Una llave conectada durante el
arranque puede detectarse antes de que exista el CSV.

Instalar en la Raspberry:

```sh
cd /home/minihawk/Payload-SDK/tools/minihawk_usb_export
sudo sh install.sh
```

Tambien se puede instalar con `sh install.sh /ruta/rootfs` sobre la tarjeta
montada en otro equipo. En ese caso se activa en el siguiente arranque de la Pi.
No se ejecuta ni se recarga ningun servicio del equipo que monta la tarjeta.

Comprobar el resultado en la Pi:

```sh
journalctl -u 'minihawk-usb-export@*' -n 30 --no-pager
```

Esperar el mensaje `COPIA COMPLETA` antes de retirar la llave. Si el escritorio
tambien la monto, expulsarla desde el escritorio. Los errores de espacio,
desconexion o montaje quedan en el journal. Un corte brusco puede dejar un
archivo oculto `.part`; solo los archivos terminados en `.csv` son exportaciones
publicadas. La copia original permanece en la Pi.

Pruebas sin dispositivos USB ni privilegios:

```sh
python3 -m unittest discover -s tools/minihawk_usb_export -v
```

Referencia del mecanismo de activacion:
[systemd.device, SYSTEMD_WANTS](https://man7.org/linux/man-pages/man5/systemd.device.5.html).

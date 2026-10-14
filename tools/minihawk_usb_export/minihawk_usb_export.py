#!/usr/bin/python3
"""Copy a bounded snapshot of complete MiniHawk CSV rows to a USB filesystem."""

import datetime
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
import uuid

SOURCE = Path('/home/minihawk/MiniHawkData/mediciones.csv')
FILESYSTEMS = {'vfat', 'exfat', 'ntfs', 'ntfs3', 'ext2', 'ext3', 'ext4'}
CHUNK_SIZE = 65536


def snapshot(source, destination):
    """Copy only bytes present on opening, dropping an unfinished final row.

    The acquisition writer appends numeric CSV rows. No writer lock is needed:
    bytes beyond the captured size are excluded even if acquisition continues.
    """
    with source.open('rb') as reader:
        remaining = os.fstat(reader.fileno()).st_size
        if remaining == 0:
            return None
        timestamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        name = f'MicroGas_mediciones_{timestamp}_{uuid.uuid4().hex}.csv'
        final = destination / name
        temporary = destination / ('.' + name + '.part')
        if os.path.lexists(final):
            raise FileExistsError(final)
        # Exclusive creation also prevents following a pre-existing symlink.
        with temporary.open('xb') as writer:
            try:
                written = complete = newlines = 0
                while remaining:
                    chunk = reader.read(min(CHUNK_SIZE, remaining))
                    if not chunk:
                        raise OSError('El CSV se redujo durante la copia')
                    writer.write(chunk)
                    end = chunk.rfind(b'\n')
                    if end >= 0:
                        complete = written + end + 1
                    newlines += chunk.count(b'\n')
                    written += len(chunk)
                    remaining -= len(chunk)
                if newlines < 2:  # Header alone is not a measurement.
                    return None
                writer.truncate(complete)
                writer.flush()
                os.fsync(writer.fileno())
            except BaseException:
                temporary.unlink(missing_ok=True)
                raise
            finally:
                if newlines < 2:
                    temporary.unlink(missing_ok=True)
        # The UUID keeps exports distinct even when the Pi clock is not set.
        try:
            os.rename(temporary, final)
            directory = os.open(destination, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
        return final, newlines - 1


def validate_device(name):
    if not re.fullmatch(r'sd[a-z]+[0-9]*', name):
        raise ValueError('Nombre de dispositivo USB no valido')
    device = Path('/dev') / name
    info = device.lstat()
    if not stat.S_ISBLK(info.st_mode):
        raise ValueError('El destino no es un dispositivo de bloques')
    properties = subprocess.run(
        ['udevadm', 'info', '--query=property', '--name', str(device)],
        check=True, capture_output=True, text=True,
    ).stdout
    properties = dict(line.split('=', 1) for line in properties.splitlines() if '=' in line)
    if properties.get('ID_BUS') != 'usb' or properties.get('ID_FS_USAGE') != 'filesystem':
        raise ValueError('El destino no es un sistema de archivos USB')
    if properties.get('ID_FS_TYPE') not in FILESYSTEMS:
        raise ValueError('Formato USB no admitido; usar FAT32 o exFAT')
    # Never export to a USB partition used for the Pi system or the source CSV.
    for path in (Path('/'), Path('/boot'), Path('/boot/firmware'), Path('/home'), SOURCE.parent):
        if path.exists() and path.stat().st_dev == info.st_rdev:
            raise ValueError('Se omite una particion usada por el sistema de la Pi')
    return device


def export_usb(name):
    device = validate_device(name)
    if not SOURCE.is_file():
        print('Aun no existe el CSV; conectar de nuevo la llave despues de medir.', flush=True)
        return
    # systemd creates this root-owned directory and a private mount namespace.
    mountpoint = Path('/run') / f'minihawk-usb-export-{name}' / 'drive'
    mountpoint.mkdir(mode=0o700)
    subprocess.run(['mount', '-o', 'rw,nosuid,nodev,noexec', str(device), str(mountpoint)], check=True)
    try:
        result = snapshot(SOURCE, mountpoint)
    finally:
        subprocess.run(['umount', str(mountpoint)], check=True)
        mountpoint.rmdir()
    if result is None:
        print('CSV sin mediciones completas; no se genero una copia.', flush=True)
    else:
        path, rows = result
        print(f'COPIA COMPLETA: {path.name}; {rows} filas. Montaje de exportacion desmontado.', flush=True)


def interrupted(signum, frame):
    raise InterruptedError('Exportacion interrumpida')


def main():
    if len(sys.argv) != 2:
        print('Uso: minihawk-usb-export <dispositivo, por ejemplo sda1>', file=sys.stderr)
        return 2
    signal.signal(signal.SIGTERM, interrupted)
    try:
        export_usb(sys.argv[1])
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'ERROR DE COPIA USB: {error}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

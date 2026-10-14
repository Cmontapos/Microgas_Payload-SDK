#!/bin/sh
set -eu

# Optional argument installs into a mounted Raspberry root filesystem.
target_root=${1:-/}
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test -f "$target_root/etc/os-release"
install -D -m 0755 "$script_dir/minihawk_usb_export.py" "$target_root/usr/local/sbin/minihawk-usb-export"
install -D -m 0644 "$script_dir/minihawk-usb-export@.service" "$target_root/etc/systemd/system/minihawk-usb-export@.service"
install -D -m 0644 "$script_dir/90-minihawk-usb-export.rules" "$target_root/etc/udev/rules.d/90-minihawk-usb-export.rules"

if [ "$target_root" = / ]; then
    systemctl daemon-reload
    udevadm control --reload-rules
    echo 'Instalado. Conectar de nuevo la llave USB para exportar el CSV.'
else
    echo "Instalado en $target_root. Se activara al iniciar la Raspberry."
fi

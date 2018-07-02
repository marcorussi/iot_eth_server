#!/bin/bash

read -d '' USAGE <<- EOF
megaprog.sh
This tool relies on OpenOCD to interface with the STLink hardware.
usage:
megaprog.sh <action> [hexfile]
where action is one of
  --flash-hex
EOF

HEX=$2

OPENOCD_PATH="/opt/openocd/0.10.0-7-20180123-1217"
OPENOCD_EXE="${OPENOCD_PATH}/bin/openocd"
OPENOCD_OPTIONS="-f ${OPENOCD_PATH}/scripts/interface/stlink.cfg -f ${OPENOCD_PATH}/scripts/board/stm32f4discovery.cfg"
OPENOCD_CMDS="-c \"program ${HEX} verify reset exit\" "

OPENOCD_FULL="$OPENOCD_EXE $OPENOCD_OPTIONS $OPENOCD_CMDS"

TMPSCRIPT=tools/tmp_$$.openocd

if [ "$1" = "--flash-hex" ]; then
    echo $OPENOCD_FULL >> $TMPSCRIPT
    chmod +x $TMPSCRIPT
    echo "Flashing ${HEX}..."
    $TMPSCRIPT
    rm $TMPSCRIPT
else
    echo "$USAGE"
fi

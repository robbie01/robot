#!/bin/bash

echo "Robbie's S19-to-SD for Linux v1.0"
echo "---------------------------------"

if MOUNT=$(findmnt -no TARGET -S LABEL=FEHSD); then
    FILE=${MOUNT}/CODE.S19
    echo "Removing old S19 files"
    rm -f "${MOUNT}/*.S19" || exit $?
    echo "Copying CODE.S19 to SD card"
    cp *.s19 "$FILE" || exit $?
    echo "Syncing filesystem"
    sync -f "$FILE"
else
    echo "SD card is not mounted."
fi

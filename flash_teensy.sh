#!/usr/bin/env bash

cd src/linorobot2_hardware/firmware
pio run --target upload -e polybot

cd ../../
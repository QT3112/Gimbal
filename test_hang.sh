#!/bin/bash
arm-none-eabi-gdb -nx --batch \
  -ex 'target extended-remote | openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "gdb_port pipe"' \
  -ex 'monitor halt' \
  -ex 'bt' \
  -ex 'monitor resume' \
  -ex 'quit'

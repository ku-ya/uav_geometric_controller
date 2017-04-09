#!/usr/bin/env python
import numpy as np
from smbus import SMBus

print('i2c test')

adress = 0x40

i2c = SMBus(1)
i2c.write_byte(adress,0)



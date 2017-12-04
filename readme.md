![Build Status](https://travis-ci.org/frankleonrose/ParameterStore.svg?branch=master)

Reliable storage of named parameters in non-volatile memories. Supports Adafruit FRAM and Bluetooth NVRAM storage and creation of alternative storage adapters. Parameter values may be basic numeric types, strings, byte arrays, or structs. Storing values by name results in less tight coupling between parameter values. Supports CRC error checking so that you can respond to bit drift. Also supports power-safe writes that will return a prior value if the mcu lost power while writing. Multiple values may be written within an atomic transaction - all succeed or all revert to prior values.

## Features


## API


## Adding Storage Adapters



## Tests

When you submit pull requests, please include tests.

Copyright (c) 2017 Frank Leon Rose
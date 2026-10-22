Bosch BMA580 SensorAPI v4.2.0
Source: https://github.com/boschsensortec/BMA580_SensorAPI
Commit: 79d2720b9a6f9f04703ade798f5621187ff8c928

The eight .c/.h files in src/bosch/ are copied without modification.
Original copyright notices and BSD-3-Clause license are retained in LICENSE.
Arduino wrapper version: 1.1.0 (includes motion interrupt support).
The Arduino wrapper is also distributed under BSD-3-Clause.

Transport supports I2C register transactions up to 32 bytes read / 31 bytes
write, for compatibility with Arduino Wire's minimum buffer size. Advanced
Bosch FIFO/feature transfers larger than these limits return a communication
error; this wrapper does not implement streaming FIFO or SPI transport.

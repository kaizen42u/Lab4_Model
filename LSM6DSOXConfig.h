#pragma once // Ensures the file is included only once during compilation to prevent multiple definitions

#define IMU_SAMPLING_RATE 104      // Defines the sampling rate for the IMU sensors. Options are: 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333 and 6667 Hz.
#define IMU_FIFO_WATERMARK_LEVEL 2 // Defines the FIFO watermark threshold level. The maximum number of samples in this FIFO configuration is 512 (for accelerometer and gyroscope only).
#define IMU_ACCELEROMETER_SCALE 4  // Defines the accelerometer scale. Available values are: 2, 4, 8, 16 G (G-forces).
#define IMU_GYROSCOPE_SCALE 2000   // Defines the gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 DGS (degrees per second).

// ---------------------------------------
// The following defines a subset of IMU FIFO Tags. Do not change.

#define IMU_FIFO_TAG_GYROSCOPE 1     // Defines the FIFO tag to indicate that the FIFO stores gyroscope data.
#define IMU_FIFO_TAG_ACCELEROMETER 2 // Defines the FIFO tag to indicate that the FIFO stores accelerometer data.

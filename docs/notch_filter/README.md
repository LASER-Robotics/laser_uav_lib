# NotchFilter

`laser_uav_lib::NotchFilter` attenuates a selected frequency band in a scalar signal. It is used to reduce periodic vibration, such as motor-related components in IMU measurements, and runs an IIR filter internally.

Include [laser_uav_lib/filter/notch_filter.hpp](../../include/laser_uav_lib/filter/notch_filter.hpp) and link `notch_filter`.

## Use

```cpp
#include <laser_uav_lib/filter/notch_filter.hpp>

laser_uav_lib::NotchFilter filter(1000.0, 60.0, 5.0);
double filtered_sample = filter.iterate(1.0);
```

This configures a 60 Hz notch with a 5 Hz bandwidth for a signal sampled at 1000 Hz. Keep the instance between samples; each independent signal needs its own filter history.

## Parameters and methods

| Argument or method | Meaning |
| --- | --- |
| `sample_rate` | Input sampling rate in Hz. |
| `frequency_in` | Frequency to attenuate in Hz. |
| `bandwidth_in` | Width of the attenuated band in Hz. |
| `iterate(sample)` | Processes one sample and returns the filtered value. |
| `getInternalFilterCoeffs()` | Returns the internal IIR feedback and feedforward coefficient vectors as a pair. |

Choose the notch from the measured vibration spectrum and keep its band within the range supported by the sampling rate. For several notch frequencies on IMU data, use [ImuFilter](../imu_filter/README.md).

[Library overview](../../README.md)

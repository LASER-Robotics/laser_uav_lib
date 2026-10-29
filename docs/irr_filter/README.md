# IIRFilter

`laser_uav_lib::IIRFilter` filters a scalar signal one sample at a time. It combines current and previous inputs with previous outputs, using coefficients supplied by the caller. LUS uses it to condition sensor and control-feedback signals.

Include [laser_uav_lib/filter/irr_filter.hpp](../../include/laser_uav_lib/filter/irr_filter.hpp) and link `irr_filter`. The class is spelled `IIRFilter`; the header and build target use `irr`.

## Use

```cpp
#include <laser_uav_lib/filter/irr_filter.hpp>

laser_uav_lib::IIRFilter filter({1.0, -0.9}, {0.1});
double filtered_sample = filter.iterate(1.0);
```

Keep the same filter instance for successive samples so that its history is preserved. Use a separate instance for each independent signal.

## Coefficients and methods

| Argument or method | Meaning |
| --- | --- |
| `a` | Feedback coefficients passed to the constructor. Normalize the coefficients for `a[0] = 1`; the implementation skips this entry and does not divide by it. |
| `b` | Feedforward coefficients passed to the constructor. |
| `iterate(input)` | Processes one sample and returns the filtered value. |
| `getCoeffs()` | Returns a tuple containing the stored `a` and `b` vectors. |
| `getInputBuffer()`, `getOutputBuffer()` | Returns the input and output history, newest sample first. |

Supply nonempty coefficient vectors designed for the actual sampling rate. The shorter vector is padded with zeros. The default constructor creates a pass-through filter.

[Library overview](../../README.md)

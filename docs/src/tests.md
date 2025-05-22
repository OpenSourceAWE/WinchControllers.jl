```@meta
CurrentModule = WinchControllers
```

# Tests
All tests can be executed using the command:
```
include("examples/menu.jl")
```
Most of these tests are not yet unit tests.

## SpeedController

### SpeedController1
This test assumes a wind speed that starts at zero and reaches its nominal value after 0.25s. The nominal
value is a triangle signal between 4 and 8 m/s and a period time of 4s. The tether direction is aligned with the 
wind direction, the set value for the reel-out speed is 4 m/s. This means when the nominal reel-out speed is reached,
the apparent wind speed is between zero and 4 m/s. The force is proportional to the square of $v_a$.

![test_speedcontroller1](assets/test_speedcontroller1.png)

### SpeedController2
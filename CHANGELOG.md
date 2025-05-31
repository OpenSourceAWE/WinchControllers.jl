### Unreleased
#### Added
- the functions `get_v_set()`, `get_f_err()`
- the script `autotune.jl`, using the **NOMAD** optimizer
- added a documentation page for the autotuning feature

### WinchControllers v0.5.2
#### Added
- the script `test_components.jl`
#### Changed
- export `saturate()`

### WinchControllers v0.5.1 2025-05-28
#### Added
- all public functions document
- add page `Performance Indicators` to documentation
- add page `Tests` to documentation
- add `WCLogger` for logging
- add `menu.jl` to the folder examples
- the functions `f_err()`, `v_err()` and `gamma()` to calculate the performance indicators of the winch controller based on the log file of a test case 

#### Changed
- reduced `df_high` in `wc_settings.yaml` to reduce oscillations
- all examples are making use of `wc_settings.yaml` now
- the function `get_v_err` returns now `NaN` instead of zero when the speed controller is inactive
- improved the plots of the examples

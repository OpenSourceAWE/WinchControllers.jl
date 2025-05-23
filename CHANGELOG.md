### WinchControllers v0.6 2025-05-23
#### Added
- all public functions document
- add page `Tests` to documentation
- add `menu.jl` to the folder examples

#### Changed
- reduced `df_high` in `wc_settings.yaml` to reduce oscillations
- all examples are making use of `wc_settings.yaml` now
- the function `get_v_error` returns now `NaN` instead of zero when the speed controller is inactive
- improved the plots of the examples

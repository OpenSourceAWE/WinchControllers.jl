### WinchControllers v0.5.6 2026-08-14
#### Added
- REEL_OUT winch-controller mode: `WCSettings.mode` (`"piecewise"`/`"reelout"`), dispatched in
  `calc_vro`; `test = true` still selects `"reelout"` for backwards compatibility
- the examples `test_reelout.jl` (bench-test plot of the REEL_OUT law) and `test/test_reelout.jl`
  (headless tracking test), both added to `menu.jl`/`menu2.jl`
- `menu()` and `menu2()` functions defined by `bin/run_julia`, bringing up `examples/menu.jl` /
  `examples/menu2.jl` in an interactive session
- `bin/install` script
- default manifests `Manifest-v1.11.toml.default` and `Manifest-v1.12.toml.default`
- CI badge in `README.md`
#### Changed
- switched from `ControlPlots` to `MakieControlPlots` throughout `examples/`, `test/`, `mwes/` and
  the `src/` docstrings
- `bin/run_julia`: forwards arguments to `julia` and `cd`s to the repo root when invoked from `bin/`
- improved `bin/install`
- CI workflow fixes

### WinchControllers v0.5.5 2026-03-23
#### Added
- `CITATION.cff` file
- `.markdownlint` configuration file

### WinchControllers v0.5.4 2026-03-03
#### Added
- the package `RobustAndOptimalControl`
- the example `stability_ufc.jl` which tests the stability of the upper force controller
#### Changed
- bump KiteUtils to 0.11.1
- use subprojects instead of TestEnv
- fix warnings

### WinchControllers v0.5.3 2025-05-31
#### Added
- the functions `get_v_set()`, `get_f_err()`
- the script `autotune.jl`, using the **NOMAD** optimizer
- added a documentation page for the autotuning feature
- added the function `install_examples()` and updated the docu accordingly

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

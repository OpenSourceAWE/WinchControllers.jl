### Changed
- `bin/install` only installs the tracked default manifest, instantiates and precompiles; it
  no longer sets the juliaup default, adds Revise globally or runs the tests. New flags: `-y`
  (no prompt, the `julia` on the PATH), `--update` (update the packages instead) and `-h`
- `bin/install` needs Julia 1.12 or newer; `Manifest-v1.11.toml.default` is removed

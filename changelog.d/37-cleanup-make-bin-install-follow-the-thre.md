### Changed
- `bin/install` only installs the tracked default manifest, instantiates and precompiles; it
  no longer sets the juliaup default, adds Revise globally or runs the tests. `-y` uses the
  `julia` on the PATH unless `--version X.Y` names one; `bin/update_default_manifests` now
  runs the tests itself before it writes a default manifest

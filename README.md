# Beam locomotor — browser simulation

Interactive demonstration of a flexible, skate-constrained beam driven by one
internal rotor. Two browser Workers integrate the same mechanical system with
positive- and negative-propulsion reference pairs. Clicks set common targets;
the paths are calculated live, not played from saved trajectories.

**Interactive review copy.** Open the website link supplied with this repository,
or run the files locally as described below.

## Run locally

Serve this directory over HTTP, for example `python -m http.server 8000`, then
open `http://localhost:8000/`. No installation, build, backend, analytics or CDN
is needed by visitors to the live site. ES-module Workers require HTTP/HTTPS,
not a `file://` URL. Current desktop Chrome is tested; performance depends on
the visitor's device. A narrow mobile layout is provided.

## Files and method

- `core.js`: Float64 geometry, mass/bias, energy, Cholesky solves and condensed
  implicit midpoint with Jacobian reuse and Broyden updates.
- `session.js`: original scalar controller, guidance memories, bounded recovery,
  SE(2) reconstruction, power accounting and finite observation diagnostics.
- `worker.js`: a separate simulation worker per progression reference.
- `model-data.json`: four-profile basis, quadrature, constants, physical scales,
  controller settings and reference waveforms exported from the Python model.
- `app.js`: paired accepted-state buffer, shared physical clock and interpolated
  canvas rendering. Rotor phase is unwrapped and never slowed independently.

The model-time step is 0.004 (approximately 2.28 ms physically), Newton tolerance
1e-11. Retry/subdivision guards and saturation are unchanged from the Python
demonstrator. Identifying source paths are omitted in this review copy. No Python,
NumPy, SciPy, Numba or llvmlite code is bundled or executed in the browser.

## Checks and limitations

The initial browser port was compared against the frozen Python demonstrator
on five kernel cases and eight trajectories (both signs; nominal capture,
retargeting and waypoints). All passed the thresholds fixed before comparison;
the largest spatial discrepancy at recorded checkpoints was below 8.5e-9 beam
lengths, and controller transition times matched. Guard tests include atomic
rollback after failed Newton recovery. Chrome tests cover shared timing,
controls, exports and desktop/narrow layouts.

This is a finite reduced-model numerical demonstration, not hardware evidence,
an obstacle-avoidance system, a continuum-convergence claim or a global
navigation/stopping guarantee. Portability comparisons do not establish the
instantaneous-rate accuracy of midpoint integration; the accompanying article's
resolved numerical checks are separate. A new arbitrary target is exploratory.
Rotor and wheel drawing sizes are enlarged for legibility, not physical model
parameters. Manuscript in preparation; no acceptance or publication is implied.

## Licence and provenance

Original browser implementation, interface and accompanying numerical data:
MIT, see `LICENSE.txt`, released with the copyright holders’ authorization. The browser
port was developed with AI coding assistance and checked against the Python
reference. No third-party runtime source, institutional logos, paper templates,
private correspondence or credentials are included. This licence covers this
demonstrator directory, not other website content or the article manuscript.

## Anonymous-host packaging

`index.html` embeds the same numerical data, styles and simulation sources.
Its two Blob workers require no cross-origin downloads in the anonymous
host’s sandbox. This changes loading only, not the solver or controller.
`modular.html` retains the ordinary HTTP module-worker entry point.
The hosted page displays run diagnostics as selectable JSON because its
sandbox forbids file downloads. The ordinary local page retains file export.
The simulator sends no run data; hosting infrastructure may collect access diagnostics.

# Model Equation Studio

`Model Equation Studio` is a native C++ desktop app that loads an OBJ model and fits a mathematical representation to it.

It is designed as a practical approximation tool:

- Loads `OBJ` meshes.
- Renders the model in a lightweight OpenGL viewport.
- Supports `plane`, `sphere`, `cylinder`, implicit polynomial, and `RBF` fitting.
- Keeps the main UI minimal with `Insert OBJ`, `Analyze`, and `Marching Cubes`, and moves advanced options into a `Settings` tab.
- Uses background fitting so the UI remains responsive on modest hardware.

There are now two desktop builds:

- `ModelEquationStudio_full.exe`: keeps the OBJ at full resolution.
- `ModelEquationStudio_downscaled.exe`: automatically downsamples large OBJ files to `1500` vertices before analysis.

## What "convert to an equation" means here

Arbitrary 3D meshes usually cannot be represented exactly by a single short equation.

Depending on the selected conversion mode, the app can produce:

- A best-fit plane
- A best-fit sphere
- A best-fit cylinder
- A quadratic or cubic implicit polynomial
- An RBF implicit surface

For arbitrary shapes, the fallback is an approximate implicit surface:

`f(u, v, w) = 0`

where:

- `u = (x - cx) / s`
- `v = (y - cy) / s`
- `w = (z - cz) / s`

`cx`, `cy`, `cz`, and `s` are computed from the mesh bounds so the fit is numerically stable.


## Notes

- Smooth or organic meshes generally fit better than sharp-edged models.
- A cubic fit usually captures more detail than a quadratic fit.
- Organic meshes generally fit best with the `RBF` method.
- The implicit polynomial residuals are algebraic fit metrics, not exact geometric distances.
- `Marching Cubes` builds a reconstructed preview mesh from the current fit and shows it in the viewport.

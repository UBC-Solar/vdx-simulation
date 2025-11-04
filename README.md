# vdx-simulation
The repository for all things vehicle dynamics (VDX) at [UBC Solar](https://github.com/UBC-Solar)! This repo holds simulation, calculation, and data analysis files for the past, present, and future development of our suspension, steering, and braking systems.

## Directories
|          | Tracked | Added to Search Path | Purpose                                                       |
| -------- | ------- | -------------------- | ------------------------------------------------------------- |
| indev/   | ✗       | ✗                    | local development, potentially with name collisions           |
| local/   | ✗       | ✓                    | private files such as .env, local development of dependencies |
| sandbox/ | ✓       | ✗                    | one-off calculations, sharing rudimentary scripts             |
| others   | ✓       | ✓                    | orderly, documented, reusable scripts (most things)           |

## Live Scripts
Moving forward, `vdx-simulation` has deprecated binary `.mlx` live scripts in favour of R2025a `.m` [plain text live scripts](https://www.mathworks.com/help/matlab/matlab_prog/plain-text-file-format-for-live-scripts.html). This will keep our codebase more maintainable and make editing, merges, and blame easier.

All live scripts commited to the repository should first have **Clear All Output** ran in MATLAB to keep diff sizes small. In future, a pre-commit hook will be set up to lint for un-cleared output.

## vdx_launch.m
`vdx_launch.m` is a startup utility to set up your [MATLAB session path](https://www.mathworks.com/help/matlab/matlab_env/files-and-folders-that-matlab-accesses.html) to include the full repository (while excluding `sandbox/` and `indev/`). Additionally, it prints Git status info to the terminal on startup.

To configure it as intended, set up a Windows desktop shortcut with the following:
> **Target:** `matlab -r "run('vdx_launch.m')"`  
> **Start In:** `C:/Users/YOUR_REPO_DIR`

Each time you launch MATLAB from this shortcut, you will be greeted as having entered the VDX workspace—indicating that your path is set up!

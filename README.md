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

## startup.m
`startup.m` is a MATLAB [startup file](https://www.mathworks.com/help/matlab/ref/startup.html) that automatically runs when MATLAB is launched from the repository root. It seamlessly sets up your [session path](https://www.mathworks.com/help/matlab/matlab_env/files-and-folders-that-matlab-accesses.html) to include to include cetain folders. It also runs fetches remote and prints `git status` on startup for convenience.

> [!IMPORTANT]
> Many scripts rely on dependencies in this repository, and expect them to be on your session path. As such, downloading individual scripts and attempting to run them will be unlikely to work. [Clone](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository) the whole repository.

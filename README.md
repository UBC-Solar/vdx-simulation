# vdx-simulation
Lorem ipsum

## vdx_launch.m
`vdx_launch.m` is a startup utility to set up your [MATLAB session path](https://www.mathworks.com/help/matlab/matlab_env/files-and-folders-that-matlab-accesses.html) to include the full repository (while excluding `sandbox/` and `indev/`). Additionally, it prints Git status info to the terminal on startup.

To configure it as intended, set up a Windows desktop shortcut with the following:
> **Target:** `matlab -r "run('vdx_launch.m')"`  
> **Start In:** `C:/Users/YOUR_REPO_DIR`

Each time you launch MATLAB from this shortcut, you will be greeted as having entered the VDX workspace—indicating that your path is set up!

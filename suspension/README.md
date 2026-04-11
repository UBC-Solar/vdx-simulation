# VDX Suspension

## Hardpoint Forces
Run `mainHardpointForces_Cascadia.m` to obtain the current hardpoint forces based on the geometry pulled from SW Hardpoints on [V4 Vehicle Dynamics](https://docs.google.com/spreadsheets/d/1C9o_FGi18w9rInHHT_hzzjyN-1102uIo5yY6cqIHfXk/edit?gid=1104959099#gid=1104959099) Google sheet.
Note, the main script is dependent on the other .m files in this folder and other files in `vdx-simulation`. Working in `vdx-simulation` should automatically run a `startup.m` (Munro's startup utility) to ensure proper path setup; it can also be run manually.  In a pinch, download all the .m files that are referenced in the main script and have them in your working folder with the main script.

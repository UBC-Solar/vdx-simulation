function startup()
% STARTUP  Initialize the VDX simulation workspace.
%
%   Adds all relevant repository folders to the MATLAB path,
%   then fetches from remote and prints git status.

% --- SETUP PATH ---

% retrieve path and set directory
repoRoot = fileparts(mfilename('fullpath'));
cd(repoRoot);

% build exclusion list
exclude = {'.git', 'indev', 'sandbox'};

% add repo scripts to MATLAB path (for this session)
allFolders = strsplit(genpath(repoRoot), pathsep);
keptFolders = allFolders(~cellfun(@(f) any(contains(f, exclude)), allFolders));
addpath(keptFolders{:});

clc
fprintf('Welcome to the VDX workspace!\n');

% --- VERSION CONTROL INFO  ---

[fetchStatus, fetchMsg] = system('git fetch');
if fetchStatus ~= 0
    fprintf('\nGit fetch failed:\n\t%s\n', fetchMsg);
end

[~, statusMsg] = system('git status -sb');
fprintf('\n%s\n', statusMsg);

end

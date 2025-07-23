function vdx_launch()
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

fprintf('Welcome to the VDX workspace!\n');

% --- PREPARE GIT INFO ---

% fetch latest info from remote
[fetchStatus, fetchMsg] = system('git fetch');
if fetchStatus ~= 0
    fprintf('Git fetch failed:\n%s\n\t', fetchMsg);
    return
end

% get current branch
[~, branch] = system('git rev-parse --abbrev-ref HEAD');
branch = strtrim(branch);

% get the upstream tracking branch
[upStatus, upstream] = system('git rev-parse --abbrev-ref --symbolic-full-name @{u}');
if upStatus ~= 0
    fprintf("No upstream tracking branch found for '%s'.\n", branch);
    return
end
upstream = strtrim(upstream);

% compare local branch with upstream
[cmpStatus, cmpOut] = system(sprintf('git rev-list --left-right --count %s...%s', upstream, branch));
if cmpStatus ~= 0
    fprintf('Unable to compare local branch with remote.\n');
    return
end
nums = sscanf(cmpOut, '%d\t%d');
behind = nums(1);
ahead = nums(2);

% print summary
if behind > 0 && ahead == 0
    fprintf("Remote has %d new commit(s). Run 'git pull' to update.\n", behind);
elseif ahead > 0 && behind == 0
    fprintf("You have %d local commit(s) not pushed to remote. Run 'git push'.\n", ahead);
elseif ahead > 0 && behind > 0
    fprintf("Your branch has diverged: %d ahead, %d behind.\nRun 'git pull' and resolve any conflicts before pushing", ahead, behind);
else
    fprintf("Local branch '%s' is up to date with remote.\n", branch);
end

end

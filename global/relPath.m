function p = relPath(folderSlashFile)
%RELPATH  Resolve path relative to the caller's file location.
%   p = RELPATH(relpth) returns an absolute path built from 'rel',
%   interpreted relative to the .m file that invoked RELPATH.
%
%   Used to make the repository directory agnostic—the working
%   directory should not alter script behaviour.

arguments
    folderSlashFile string
end

s = dbstack('-completenames');
if numel(s) < 2
    error('relPath must be called from within a script or function.');
end
callerFile = s(2).file;
callerFolder = fileparts(callerFile);

% build absolute path
p = fullfile(callerFolder, folderSlashFile);
end


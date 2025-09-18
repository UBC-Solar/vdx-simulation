% demo of the relPath() function found in global/

CSV = relPath('subfolder/example.csv');

data = readmatrix(CSV);

fprintf('Contents at %s:\n%s', CSV);
disp(data)

function ts = loadTimestamps(ts_dir)

% ts_dir='/home/amar/Documents/MATLAB/Combined VisualSoft and IMU';
fid = fopen([ts_dir '/timestamps.txt'],'r+');
col = textscan(fid,'%s\n',-1,'delimiter','/n');
ts = col{1};
fclose(fid);


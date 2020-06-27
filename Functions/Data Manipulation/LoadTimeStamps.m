function [ts] = LoadTimeStamps(ts_dir)
%LOADTIMESTAMPS Returns TimeStamp Data
%   Pass In the folder with the KITTI time stamp data in,
%   and this function will parse it to return the timeStamps.
fid = fopen([ts_dir '/timestamps.txt']);
col = textscan(fid,'%s\n',-1,'delimiter',',');
ts = col{1};
fclose(fid);
end


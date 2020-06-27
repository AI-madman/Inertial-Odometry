function [GPS] = LoadGPS(GPSFile,numOfFrames)
%LOADGPS Returns a Strut of GPS Data
%   The GPS Struct has 3 axis of infomation (X,Y,Z).
%   For each axis the man/min and postion data is given.
  ground_truth = load(GPSFile);
 
  for i=1:1:numOfFrames-1
        T = reshape(ground_truth(i, :), 4, 3)';
        pos_gt = T(:, 4);
        GPS.X(i)=pos_gt(1);
        GPS.Y(i)=pos_gt(2);
        GPS.Z(i)=pos_gt(3);
  end
  GPS.max.y=max(GPS.Y);
  GPS.min.y=min(GPS.Y);
  GPS.max.x=max(GPS.X);
  GPS.min.x=min(GPS.X);
  GPS.max.z=max(GPS.Z); 
  GPS.min.z=min(GPS.Z);
end


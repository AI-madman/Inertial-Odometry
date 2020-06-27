function [Tr,R,T] = loadCalibrationRigid(filename)
% Returns Homogenous Rigid Body Transformation
% Made for KITTI Dataset, pass in the file containing the 
% Rotation and Translation between refrance frames and get 
% back the transformation.
% This function has been edited to return all three.
fid = fopen(filename,'r');

if fid<0
  error(['ERROR: Could not load: ' filename]);
end

% read calibration
R  = readVariable(fid,'R',3,3);
T  = readVariable(fid,'T',3,1);
Tr = [R T;0 0 0 1];

fclose(fid);


function A = readVariable(fid,name,M,N)

% rewind
fseek(fid,0,'bof');

% search for variable identifier
success = 1;
while success>0
  [str,success] = fscanf(fid,'%s',1);
  if strcmp(str,[name ':'])
    break;
  end
end

% return if variable identifier not found
if ~success
  A = [];
  return;
end

% fill matrix
A = zeros(M,N);
for m=1:M
  for n=1:N
    [val,success] = fscanf(fid,'%f',1);
    if success
      A(m,n) = val;
    else
      A = [];
      return;
    end
  end
end


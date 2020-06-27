GTD.X=zeros(1,NumOfImages);
GTD.Y=zeros(1,NumOfImages);
GTD.Z=zeros(1,NumOfImages);
for i=1:1:1050
   GTD.X(i)=groundTruthStates{i}.imuState.p_I_G(1);
   GTD.Y(i)=groundTruthStates{i}.imuState.p_I_G(2);
   GTD.Z(i)=groundTruthStates{i}.imuState.p_I_G(3);
   DX=GTD.X(i)-GPSData.X(i);
   if DX~=0
       disp('DX ~=0')
   end
   DY=GTD.Y(i)-GPSData.Y(i);
   if DX~=0
       disp('DY ~=0')
   end
   DZ=GTD.Z(i)-GPSData.Z(i);
   if DX~=0
       disp('DZ ~=0')
   end
end

%  for i=1:1:size(groundTruthStates,2)
% state=groundTruthStates{i}.imuState.p_I_G;
% end
scatter3(GTD.X,GTD.Y,GTD.Z);
hold on;
scatter3(GPSData.X,GPSData.Y,GPSData.Z);
legend on;
legend('IMU','GPS')
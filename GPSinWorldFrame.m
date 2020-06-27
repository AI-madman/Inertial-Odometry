IMUPostion=zeros(3,1057);
for i=1:1:1057
    IMUPostion(:,i)=KFState_IMUOnly{i}.imuState.p_I_G;
end
GPS_in_World_RF=zeros(3,NumOfImages);
for ii=1:1:length(GPSData.X)
    GPS_in_World_RF(:,ii)=C_IG*KFState_IMUOnly{1, ii}.imuState.p_I_G;
end
GFoR=figure();
figure(GFoR)
hold on
scatter3(GPS_in_World_RF(1,:),GPS_in_World_RF(2,:),GPS_in_World_RF(3,:));
scatter3(IMUPostion(1,:),IMUPostion(2,:),IMUPostion(3,:))
title('Plot of IMU and GPS Trajectory in the world frame of refrance');
legend on
legend('GPS World Frame','IMU Odometry');

for ii=1:1:1057
InertialResults.Error.Euclidain.X(1,ii)=abs((GPS_in_World_RF(1,ii)-IMUPostion(1,ii)));
InertialResults.Error.Euclidain.Y(1,ii)=abs((GPS_in_World_RF(2,ii)-IMUPostion(2,ii)));
InertialResults.Error.Euclidain.Z(1,ii)=abs((GPS_in_World_RF(3,ii)-IMUPostion(3,ii)));
InertialResults.Error.Euclidain.XYZ(1,ii)=InertialResults.Error.Euclidain.X(1,ii)+InertialResults.Error.Euclidain.Y(1,ii)+InertialResults.Error.Euclidain.Z(1,ii);

end

InertialErrorPlots=figure();
figure(InertialErrorPlots)
title({'Plot of the errors', 'in the inertial odometry'});
subplot(1,4,1)
plot(InertialResults.Error.Euclidain.XYZ);
xlabel('XYZ Error');
ylabel('Error in Meters');
subplot(1,4,2)
plot(InertialResults.Error.Euclidain.X);
xlabel('X Error');
ylabel('Error in Meters');
subplot(1,4,3)
plot(InertialResults.Error.Euclidain.Y);
xlabel('Y Error');
ylabel('Error in Meters');
subplot(1,4,4)
plot(InertialResults.Error.Euclidain.Z);
xlabel('Z Error');
ylabel('Error in Meters');

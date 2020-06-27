%% Constructing the VO Pose
load(Path.VO);
VO=cell(1,NumOfImages);
for i=1:1:NumOfImages
   VO{i}.p_C1_G=zeros(3,1);
   VO{i}.q_C1_G=zeros(4,1);
end
for i=1:1:1100
    VO{i}.p_C1_G=[Postion.estimate.Odometry.x(i) Postion.estimate.Odometry.y(i) Postion.estimate.Odometry.z(i)]';
    VO{i}.q_C1_G=RotationMatrix2Quaterion(Postion.transtion.Odometry.rotation(:,:,i));
end
%% Initial CamCovariance
e_x=0;
e_y=0;
e_z=0;
for i=1:1:100
e_x=e_x+Postion.estimate.Odometry.x(i)-GPSData.X(i);
e_y=e_x+Postion.estimate.Odometry.y(i)-GPSData.Y(i);
e_z=e_x+Postion.estimate.Odometry.z(i)-GPSData.Z(i);
end
e_x_2=e_x*e_x;
e_y_2=e_y*e_y;
e_z_2=e_z*e_z;
confidance_x=1/e_x;
confidance_y=1/e_y;
confidance_z=1/e_z;
InitalCovariance.Cam=eye(12);
InitalCovariance.Cam(1:4,:)=InitalCovariance.Cam(1:4,:)*confidance_x;
InitalCovariance.Cam(5:8,:)=InitalCovariance.Cam(5:8,:)*confidance_y;
InitalCovariance.Cam(9:12,:)=InitalCovariance.Cam(9:12,:)*confidance_z;
%% Inital CamIMU CrossVariance
InitalCovariance.IMU=noiseParams.initialIMUCovar;
InitalCovariance.CamIMU=sqrt(InitalCovariance.IMU)*sqrt(InitalCovariance.Cam);
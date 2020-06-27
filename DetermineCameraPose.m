% Determins camera pose from IMU Pose
IMUOdometryPose.postion.C1=zeros(3,1);
IMUOdometryPose.postion.C2=zeros(3,1);
IMUOdometryPose.postion.IMU=zeros(3,1);
IMUOdometryPose.orientation.C1=zeros(4,1);
IMUOdometryPose.orientation.C1=zeros(4,1);
IMUOdometryPose.orientation.IMU=zeros(4,1);

for ii=1:1:NumOfImages
    C_IG = Quaterion2RotationMatrix(KFState{ii}.imuState.q_IG);
    
    IMUOdometryPose.orientation.C1(:,ii) = quatLeftComp(camera.q_CI) * KFState{ii}.imuState.q_IG;
    IMUOdometryPose.postion.C1 = KFState{ii}.imuState.p_I_G + C_IG' * camera.p_C_I;
    
    IMUOdometryPose.orientation.C2(:,ii) = quatLeftComp(camera.q2_CI) * KFState{ii}.imuState.q_IG;
    IMUOdometryPose.postion.C2 = KFState{ii}.imuState.p_I_G + C_IG' * camera.p2_C_I;
    
    IMUOdometryPose.orientation.IMU(:,ii)=C_IG;
    IMUOdometryPose.postion.IMU(:,ii)=KFState{ii}.imuState.p_I_G;
    
end

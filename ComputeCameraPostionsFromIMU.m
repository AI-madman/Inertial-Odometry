function KFState_aug = ComputeCameraPostionsFromIMU(KFState, camera)
% Convert IMU postion to camera postions    
% Takes in the whole state struct access the.imu substruct to determine the 
% postions of the pose of the two cameras from the IMU pose using rigid
% body transfromations and adds them to the input struct in a CamStates
% substruct.

    C_IG = Quaterion2RotationMatrix(KFState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    q_CG = quatLeftComp(camera.q_CI) * KFState.imuState.q_IG;
    p_C_G = KFState.imuState.p_I_G + C_IG' * camera.p_C_I;
    
    q2_CG = quatLeftComp(camera.q2_CI) * KFState.imuState.q_IG;
    p2_C_G = KFState.imuState.p_I_G + C_IG' * camera.p2_C_I;
    
    KFState_aug=KFState;
    KFState_aug.CamStates.C_IG=C_IG;
    KFState_aug.CamStates.q_CG=q_CG;
    KFState_aug.CamStates.p_C_G=p_C_G;
    KFState_aug.CamStates.q2_CG=q2_CG;
    KFState_aug.CamStates.p2_C_G=p2_C_G;
end
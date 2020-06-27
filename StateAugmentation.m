function[AugmentedState]=StateAugmentation(KFState, camera, i)
% Updates the State by adding IMU predictions of camera postions.

    C_IG = Quaterion2RotationMatrix(KFState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    q_CG = quatLeftComp(camera.q_CI) * KFState.imuState.q_IG;
    p_C_G = KFState.imuState.p_I_G + C_IG' * camera.p_C_I;
    
    q2_CG = quatLeftComp(camera.q2_CI) * KFState.imuState.q_IG;
    p2_C_G = KFState.imuState.p_I_G + C_IG' * camera.p2_C_I;
    
    J = CaculateJMatrix(camera, KFState.imuState);
    KFState.imuCovar=J*KFState.imuCovar*J';
    
    P_aug=[KFState.imuCovar, KFState.imuCamCovar;
       KFState.imuCamCovar', KFState.camCovar];
   
    AugmentedState.imuCovar = P_aug(1:12,1:12);
    AugmentedState.camCovar = P_aug(13:end,13:end);
    AugmentedState.imuCamCovar = P_aug(1:12, 13:end);
    AugmentedState.camStates.q2_CG=q2_CG;
    AugmentedState.camStates.p2_C_G=p2_C_G;
    AugmentedState.camStates.q_CG=q_CG;
    AugmentedState.camStates.p_C_G=p_C_G;
    AugmentedState.ID=i;

end
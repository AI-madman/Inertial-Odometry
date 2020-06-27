function KFState_aug = AugmentState(KFState, camera, i)
% Augments the state with a new camera pose    

    C_IG = Quaterion2RotationMatrix(KFState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    q_CG = quatLeftComp(camera.q_CI) * KFState.imuState.q_IG;
    p_C_G = KFState.imuState.p_I_G + C_IG' * camera.p_C_I;
    
    q2_CG = quatLeftComp(camera.q2_CI) * KFState.imuState.q_IG;
    p2_C_G = KFState.imuState.p_I_G + C_IG' * camera.p2_C_I;

    % Build  covariance matrix
    P = [KFState.imuCovar, KFState.imuCamCovar, KFState.imuCamCovar;
        KFState.imuCamCovar', KFState.camCovar, KFState.camCovar; 
        KFState.imuCamCovar', KFState.camCovar', KFState.camCovar]; % the covariance for 3 sensors
    

    % Camera state Jacobian
    J = CaculateJMatrix(camera, KFState.imuState);
    
    tempMat = [eye(12); J];
    
    size(tempMat)
    size(P)
    P_aug = tempMat * P * tempMat';
    
    % Break everything into appropriate structs
    KFState_aug = KFState;
    KFState_aug.camStates_L{1}.p_C_G = p_C_G;
    KFState_aug.camStates_L{1}.q_CG = q_CG;
    KFState_aug.camStates_R{1}.p2_C_G = p2_C_G; 
    KFState_aug.camStates_R{1}.q2_CG = q2_CG; 
    KFState_aug.camStates_L{1}.state_k = i;
    KFState_aug.camStates_R{1}.state_k = i;
    KFState_aug.imuCovar = P_aug(1:12,1:12);
    KFState_aug.camCovar = P_aug(13:end,13:end);
    KFState_aug.imuCamCovar = P_aug(1:12, 13:end);
end
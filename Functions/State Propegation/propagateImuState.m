function UpdatedState = propagateImuState(KFState, measurements_k)
% Computes the pose of IMU at one itteration in the futher
% Uses the mensuremts of the IMU to compute the pose of the IMU at time
% t+dT. This is done using basic equations of motion.

    C_IG = Quaterion2RotationMatrix(KFState.imuState.q_IG);
    
    
    % Rotation state
    psi = (measurements_k.omega - KFState.imuState.b_g) * measurements_k.dT;
    changedState.q_IG = KFState.imuState.q_IG + 0.5 * CaculateOmegaMatrix(psi) * KFState.imuState.q_IG;

    
    %Unit length quaternion
    changedState.q_IG = changedState.q_IG/norm(changedState.q_IG);
    
    % Bias states
    changedState.b_g = KFState.imuState.b_g;
    changedState.b_v = KFState.imuState.b_v;
    changedState.b_a = KFState.imuState.b_a;
    
    % Translation state
    d = (measurements_k.v - KFState.imuState.b_v) * measurements_k.dT;
    changedState.p_I_G = C_IG' * d + KFState.imuState.p_I_G; 
    
    UpdatedState=KFState;
    UpdatedState.imuState=changedState;
end
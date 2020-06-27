 function imuStates_up = UpdateStateHistory(imuStates, KFState, camera, k_Itteration)
% updateStateHistory -- updates IMU state history from current msckfState
%
% INPUTS:   imuStates -- all the IMU states
%           KFState -- the current KFState (at timestep k_Itteration)
%           camera -- camera parameters (needed for IMU-to-camera transform)
%           k_Itteration -- the current timestep
%
% OUTPUTS: imuStates_up -- all the IMU states, replaced
%                           by the msckfState versions where available


    imuStates_up = imuStates;

    % Update the current IMU state
    imuStates_up{k_Itteration}.q_IG = KFState.imuState.q_IG;
    imuStates_up{k_Itteration}.p_I_G = KFState.imuState.p_I_G;
    imuStates_up{k_Itteration}.b_g = KFState.imuState.b_g;
    imuStates_up{k_Itteration}.b_v = KFState.imuState.b_v;
    imuStates_up{k_Itteration}.covar = KFState.imuCovar;
    
    % Update IMU states corresponding to active camera poses
%     keyboard
%     % Camera to IMU transformation
%     C_CI = Quaterion2RotationMatrix(camera.q_CI);
%     q_IC = RotationMatrix2Quaterion(C_CI');
%     p_I_C = - C_CI' * camera.p_C_I;
% %     p_I_C = [-0.32;0.72;-1.03];
%     
%     C2_CI = Quaterion2RotationMatrix(camera.q2_CI);
%     q2_IC = RotationMatrix2Quaterion(C2_CI');
%     p2_I_C = - C2_CI' * camera.p2_C_I;
end


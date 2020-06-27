function KFState_prop = propagateKFStateAndCovar(KFState, measurements_k, noiseParams)
    
    % Jacobians
    Q_imu = noiseParams.Q_imu;
    F = CaculateFMatrix(KFState.imuState, measurements_k);
    G = CaculateGMatrix(KFState.imuState);

    %Propagate State
    KFState_prop.imuState = propagateImuState(KFState.imuState, measurements_k);

    % State Transition Matrix
    Phi = eye(size(F,1)) + F * measurements_k.dT; % Leutenegger 2013
    
    % IMU-IMU Covariance
%     msckfState_prop.imuCovar = msckfState.imuCovar + ...
%                                 ( F * msckfState.imuCovar ...
%                                 + msckfState.imuCovar * F' ...
%                                 + G * Q_imu * G' ) ...
%                                         * measurements_k.dT;

    KFState_prop.imuCovar = Phi * KFState.imuCovar * Phi' ...
                                + G * Q_imu * G' * measurements_k.dT; % Leutenegger 2013
    
    % Enforce PSD-ness
    KFState_prop.imuCovar = enforcePSD(KFState_prop.imuCovar);
                                    
    % Camera-Camera Covariance
    KFState_prop.camCovar = KFState.camCovar;
    
    % IMU-Camera Covariance
    KFState_prop.imuCamCovar = Phi * KFState.imuCamCovar;
%    KFState_prop.camStates_L = KFState.camStates_L;
%    KFState_prop.camStates_R = KFState.camStates_R;
end

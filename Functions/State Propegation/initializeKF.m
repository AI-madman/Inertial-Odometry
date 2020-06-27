function [KFState] = initializeKF(firstImuState, camera, state_k, noiseParams)
%INITIALIZEKF Constructs the First KFState
%This function is degined to make the first state for the KF. This is done
%to make it easy to use the code base on a diffrent dataset.
firstImuState.b_g = zeros(3,1);
firstImuState.b_v = zeros(3,1);
KFState.imuState = firstImuState;
KFState.imuCovar = noiseParams.initialIMUCovar;
KFState.camCovar = [];
KFState.imuCamCovar = [];
KFState.camStates_L = {};
KFState.camStates_R = {};
KFState = AugmentState(KFState, camera, state_k);

end


function[KFState,P]=propagateIMUCovariance(KFState, measurements, noiseParams)
% Propagates the IMU covariance
% Uses the state propagation functions and jacobian matrixies to encode the
% noise paramters and motion uncerinty into a single matrix: the covariance matrix.

F=CaculateFMatrix(KFState.imuState,measurements);
G=CaculateGMatrix(KFState.imuState);
Q_imu = noiseParams.Q_imu;

TransionMatrix=eye(size(F,1)) + F * measurements.dT; % Leutenegger 2013

KFState.imuCovar=TransionMatrix*KFState.imuCovar*TransionMatrix' + G*Q_imu*G'*measurements.dT;
KFState.imuCovar=enforcePSD(KFState.imuCovar);
KFState.imuCamCovar=TransionMatrix*KFState.imuCamCovar;

P=[KFState.imuCovar, KFState.imuCamCovar;
   KFState.imuCamCovar', KFState.camCovar];
end
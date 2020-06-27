%% Load In all nessacry data
ConfigInertialOdometry
if Switch.LoadVO==1
    ImportVOState
end
disp('VO States Loaded In')
%% Setup Inital State
ImuStates=cell(1,numel(IMUData.timestamps));
KFState=initializeKF(firstImuState, camera, 1, noiseParams);
ImuStates=UpdateStateHistory(ImuStates, KFState, camera,1);
KFState_IMUOnly{1} = KFState;
KFState.imuCovar = InitalCovariance.IMU;
KFState.camCovar = InitalCovariance.Cam;
KFState.imuCamCovar = InitalCovariance.CamIMU;
%% Main Loop
for i=1:1:length(GPSData.X)
    disp(strcat('Current Itteration of EKF Loop is:',int2str(i)))
    KFState_IMUOnly{i+1} = propagateImuState(KFState_IMUOnly{i}, measurements{i});
    KFState_IMUOnly{i+1} = ComputeCameraPostionsFromIMU(KFState_IMUOnly{i+1}, camera);
  
%     [KFState_IMUOnly{i+1},P]=propagateIMUCovariance(KFState_IMUOnly{i+1}, measurements{i}, noiseParams);
    
%     [IMUState2D,P]=ComputeOptimalFusion(KFState_IMUOnly{i+1},VO{i+1},P);
%     KFState_IMUOnly{i+1}=UpdateCurrentStruct(KFState_IMUOnly{i+1},IMUState2D,P);

end
if Switch.CTIMU==1
    ConstructIMUTrajecory
end
if Switch.PlotIMU==1
    GPSinWorldFrame
end

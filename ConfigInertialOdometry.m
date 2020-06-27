%% Import Code Base
clear all
close all
addpath(genpath('/home/amar/Documents/MATLAB/Inertial Odometry'));
%% Set Paths
Sequance='07';
Path.ImageLeft = '/home/amar/Desktop/KITTI VO/2011_09_30/2011_09_30_drive_0027_sync/image_00/data';
Path.ImageRight = '/home/amar/Desktop/KITTI VO/2011_09_30/2011_09_30_drive_0027_sync/image_00/data';
Path.GPS='/home/amar/Desktop/KITTI VO/poses/';
Path.base='/home/amar/Desktop/KITTI VO/2011_09_30/2011_09_30_drive_0027_sync';
Path.calib='/home/amar/Desktop/KITTI VO/Calib&times/07';
Path.oxts='/home/amar/Desktop/KITTI VO/2011_09_30/2011_09_30_drive_0027_sync/oxts';
Path.calib2='/home/amar/Desktop/KITTI VO/Calib&times/';
Path.VO='/home/amar/Documents/MATLAB/BASOFTNOCHANGE/07VO.mat';
disp('Relevent Paths Set')

%% Import Data
S1=imageSet(Path.ImageLeft);
S2=imageSet(Path.ImageRight);
NumOfImages=S1.Count;
disp(strcat(num2str(NumOfImages),' Stereo Images Succesfully Loaded'))
GPSData=LoadGPS(strcat(Path.GPS,Sequance,'.txt'),NumOfImages-6);
disp('GPS Data Loaded')
DateTimeStampsIMU=LoadTimeStamps(Path.base);
IMUData=LoadImuMeasurments(Path.base,DateTimeStampsIMU);
TimeStampsImage=LoadImageTimeStamps(Path.ImageLeft,NumOfImages);
leftImageData=loadImageData(Path.ImageLeft,NumOfImages);
dT = [0, diff(leftImageData.timestamps)];
disp('TimeStamps Loaded')
%% Swiches
Switch.CTIMU=1;      % Prerequiste for PlotIMU
Switch.PlotIMU=1;    % 0:disabled, 1:enabled
Switch.LoadVO=1;     % Gets Pose from VO path
%% Rigid Body Transforms
%GPS has same refrance frame as IMU.
[T_camvelo_struct, P_rect_cam1] = loadCalibration(Path.calib2);
RigidTransform.Velo_IMU.Homogenous = loadCalibrationRigid(fullfile(Path.calib2,'calib_imu_to_velo.txt'));
RigidTransform.Velo_IMU.Rotation=RigidTransform.Velo_IMU.Homogenous(1:3,1:3);
RigidTransform.Velo_IMU.Translation=RigidTransform.Velo_IMU.Homogenous(1:3,4);
RigidTransform.Cam_Velo.Homogenous = T_camvelo_struct{1};
RigidTransform.Cam_Velo.Rotation=RigidTransform.Cam_Velo.Homogenous(1:3,1:3);
RigidTransform.Cam_Velo.Translation=RigidTransform.Cam_Velo.Homogenous(1:3,4);
RigidTransform.Cam2_Velo.Homogenous=T_camvelo_struct{2};
RigidTransform.Cam2_Velo.Rotation=RigidTransform.Cam2_Velo.Homogenous(1:3,1:3);
RigidTransform.Cam2_Velo.Translation=RigidTransform.Cam2_Velo.Homogenous(1:3,4);
RigidTransform.Cam_Cam2.Rotation=RigidTransform.Cam_Velo.Rotation*inv(RigidTransform.Cam2_Velo.Rotation);
RigidTransform.Cam_Cam2.Translation=RigidTransform.Cam_Velo.Translation-RigidTransform.Cam2_Velo.Translation;
RigidTransform.Cam_IMU.Rotation=RigidTransform.Cam_Velo.Rotation*RigidTransform.Velo_IMU.Rotation;
RigidTransform.Cam_IMU.Translation=RigidTransform.Cam_Velo.Translation+RigidTransform.Velo_IMU.Translation;
RigidTransform.Cam2_IMU.Rotation=RigidTransform.Cam2_Velo.Rotation*RigidTransform.Velo_IMU.Rotation;
RigidTransform.Cam2_IMU.Translation=RigidTransform.Cam2_Velo.Translation+RigidTransform.Velo_IMU.Translation;
disp('Rigid Body Transfroms Loaded and/or Caculated')
%% Setting up Intrinisct Paramters
K= P_rect_cam1(:,1:3);
b_pix = P_rect_cam1(1,4);
camera.cu = K(1,3);
camera.cv = K(2,3);
camera.fu = K(1,1);
camera.fv = K(2,2);
camera.b= -b_pix/camera.fu;
%The KITTI calibration supplies the baseline in units of pixels Hereits in [m].
disp('Internal and External Paramters Loaded')
%% Setting the relative coradinates of sensors
p_v_I = [-0.80868      0.31956     -0.79972]';
p_c_v = [-0.0040698    -0.076316     -0.27178]';
C_v_I = [9.999976e-01 7.553071e-04 -2.035826e-03;-7.854027e-04 9.998898e-01 -1.482298e-02;
2.024406e-03 1.482454e-02 9.998881e-01];
C_c_v =[7.533745e-03 -9.999714e-01 -6.166020e-04;1.480249e-02 7.280733e-04 -9.998902e-01;
9.998621e-01 7.523790e-03 1.480755e-02];
p_C_I = C_v_I'*(-C_c_v'*p_c_v - p_v_I);
C_c_I = C_c_v*C_v_I;
disp('The co-ordinates of the sensors have been set up')
%% Completeing the setup of the camera struct
camera.C_C2_C1 =[9.993513e-01 1.860866e-02 -3.083487e-02;
                 -1.887662e-02 9.997863e-01 -8.421873e-03;
                  3.067156e-02 8.998467e-03 9.994890e-01];
camera.q_C2C1  = RotationMatrix2Quaterion(camera.C_C2_C1);       
camera.p_C1_C2 = [-5.37*10^-1; 4.822061*10^-3; -1.252488*10^-2]; 
camera.p_C_I   = p_C_I;
camera.q_CI    = RotationMatrix2Quaterion(C_c_I);  % 4x1 IMU-to-Camera rotation quaternion
C_c2_I         = camera.C_C2_C1*C_c_I;
camera.q2_CI   = RotationMatrix2Quaterion(C_c2_I); % 4x1 IMU-to-Camera rotation quaternion for the second cam
camera.p2_C_I  = camera.p_C_I + C_c2_I'*(-camera.p_C1_C2); % 3x1 Camera position in IMU frame for the second cam
disp('The constant Struct repasenting the camera is setup')
%% Setting up Measurments
measurements = cell(1,numel(IMUData.timestamps));
groundTruthStates = cell(1,numel(IMUData.timestamps));
T_wIMU_GT = getGroundTruth(Path.base, NumOfImages);
r_i_vk_i = NaN(3, size(T_wIMU_GT,3));
theta_vk_i = NaN(3, size(T_wIMU_GT,3));
for j = 1:size(T_wIMU_GT,3)
r_i_vk_i(:,j) = T_wIMU_GT(1:3, 4, j);
R_wIMU = T_wIMU_GT(1:3,1:3,j);
theta = acos((trace(R_wIMU)-1)*0.5);
if theta < eps
    w = zeros(3,1);
else
    w = 1/(2*sin(theta))*[R_wIMU(3,2) - R_wIMU(2,3); R_wIMU(1,3) - R_wIMU(3,1); R_wIMU(2,1) - R_wIMU(1,2)];
end
theta_vk_i(:,j) = theta*w;
end
for i=1:1:length(GPSData.X)
    measurements{i}.dT      = dT(i);                      % sampling times
%    measurements{i}.y_L     = squeeze(y_k_j(1:2,i,:));    % left camera 
    
%     measurements{i}.y_R     = squeeze(y_k_j(3:4,i,:));    % right camera 
    
    measurements{i}.omega   = IMUData.measOmega(:,i);             % ang vel
    measurements{i}.v       = IMUData.measVel(:,i);             % lin vel

    %Idealize measurements
%     validMeas = ~isnan(measurements{i}.y_L(1,:));
%     measurements{i}.y_L(1,validMeas) = (measurements{i}.y_L(1,validMeas) - camera.c_u)/camera.f_u;
%     measurements{i}.y_L(2,validMeas) = (measurements{i}.y_L(2,validMeas) - camera.c_v)/camera.f_v;
%     
%     validMeas = ~isnan(measurements{i}.y_R(1,:));
%     measurements{i}.y_R(1,validMeas) = (measurements{i}.y_R(1,validMeas) - camera.c_u)/camera.f_u;
%     measurements{i}.y_R(2,validMeas) = (measurements{i}.y_R(2,validMeas) - camera.c_v)/camera.f_v;
    %Ground Truth
     q_IG = RotationMatrix2Quaterion(AxisAngle2RotationMatrix(IMUData.measOrient(1:3,i).')); % Rotations from Global to Inertial
     p_I_G = [GPSData.X(i),GPSData.Y(i),GPSData.Z(i)]; 
%     % position from Global to Inertial frame
%     
     groundTruthStates{i}.imuState.q_IG = q_IG;
     groundTruthStates{i}.imuState.p_I_G = p_I_G;
%     
%     % Compute camera pose from current IMU pose
      C_IG = Quaterion2RotationMatrix(q_IG);
      uno=RotationMatrix2Quaterion(RigidTransform.Cam_IMU.Rotation);
      duas=RotationMatrix2Quaterion(RigidTransform.Cam2_IMU.Rotation);
      q_CG = quatLeftComp(uno) * q_IG;
      q2_CG = quatLeftComp(duas) * q_IG; % For the second cam
      p_C_G = p_I_G + C_IG' * RigidTransform.Cam_IMU.Translation;
  
      p2_C_G = p_I_G + C_IG' * RigidTransform.Cam2_IMU.Translation;  % For the second cam

    groundTruthStates{i}.camState.p_C_G=p_C_G;
    groundTruthStates{i}.camState.p2_C_G=p2_C_G;
    groundTruthStates{i}.camState.q_CG=q_CG;
    groundTruthStates{i}.camState.q2_CG=q2_CG;
end
disp('Ground Truth States Constructed')
%% Recording Structs
InertialResults.Error.Euclidain.XYZ=zeros(1,NumOfImages);
InertialResults.Error.Euclidain.X=zeros(1,NumOfImages);
InertialResults.Error.Euclidain.Y=zeros(1,NumOfImages);
InertialResults.Error.Euclidain.Z=zeros(1,NumOfImages);
InertialResults.Postion.XYZ=zeros(3,NumOfImages);
InertialResults.Postion.X=zeros(1,NumOfImages);
InertialResults.Postion.Y=zeros(1,NumOfImages);
InertialResults.Postion.Z=zeros(1,NumOfImages);
InertialResults.Rotation=zeros(3,3,NumOfImages);
InertialResults.Translation=zeros(3,NumOfImages);
disp('Error Structs Loaded')
%% Noise Paramters
% y_var can have a big impact on final result!!!!
y_var = 11^2 * ones(1,4);               % pixel coord var
noiseParams.u_var_prime = y_var(1)/camera.fu^2;
noiseParams.v_var_prime = y_var(2)/camera.fv^2;

w_var = 4e-2 * ones(1,3);               % rot vel var
v_var = 4e-2 * ones(1,3);               % lin vel var
dbg_var = 1e-6 * ones(1,3);            % gyro bias change var
dbv_var = 1e-6 * ones(1,3);            % vel bias change var
noiseParams.Q_imu = diag([w_var, dbg_var, v_var, dbv_var]);

q_var_init = 1e-6 * ones(1,3);         % init rot var
p_var_init = 1e-6 * ones(1,3);         % init pos var
bg_var_init = 1e-6 * ones(1,3);        % init gyro bias var
bv_var_init = 1e-6 * ones(1,3);        % init vel bias var
noiseParams.initialIMUCovar = diag([q_var_init, bg_var_init, bv_var_init, p_var_init]);
disp('Noise Paramters Set')
%% Setting Up Intial Condtions
% firstImuState.q_IG = [0;0;0;1]; % Quaertion of IMU in Ground Frame
% firstImuState.p_I_G = [0;0;0];  % Postion Of IMU in Ground Frame
firstImuState.q_IG = RotationMatrix2Quaterion(AxisAngle2RotationMatrix(theta_vk_i(:,1))); % Quaertion of IMU in Ground Frame
firstImuState.p_I_G = r_i_vk_i(:,1);  % Postion Of IMU in Ground Frame
firstImuState.b_g     =zeros(3,1);
firstImuState.b_a     =zeros(3,1);
disp('The Initial Condtions have been setup')
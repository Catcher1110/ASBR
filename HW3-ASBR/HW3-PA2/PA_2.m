close all;
clear;
clc;
%% Non-Noise
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config]=data_quaternion();
[n,~] =size(q_Robot_config);
[Rx1, tx1, Rx2, tx2] =  hand_eye_calibration(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config, n);
disp(Rx1);
disp(tx1);
disp(Rx2);
disp(tx2);
normR = norm(Rx1 - Rx2);
normP = norm(tx1 - tx2);
fprintf("The error measure for rotational part is: %.8f. \n", normR);
fprintf("The error measure for translational part is: %.8f \n", normP);

%% Noise Whole Data Set
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config]=data_quaternion_noisy();
[n,~] =size(q_Robot_config);
[WNRx1, WNtx1, WNRx2, WNtx2] =  hand_eye_calibration(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config, n);
disp(WNRx1);
disp(WNtx1);
disp(WNRx2);
disp(WNtx2);
WNnormR1 = norm(WNRx1 - Rx1);
WNnormP1 = norm(WNtx1 - tx1);
fprintf("The whole set error measure for rotational part with axis-angle is: %.8f. \n", WNnormR1);
fprintf("The whole set error measure for translational part with axis-angle is: %.8f \n", WNnormP1);

WNnormR2 = norm(WNRx2 - Rx2);
WNnormP2 = norm(WNtx2 - tx2);
fprintf("The whole set error measure for rotational part with quaternion is: %.8f. \n", WNnormR2);
fprintf("The whole set error measure for translational part with quaternion is: %.8f \n", WNnormP2);

%% Noise Half Data Set
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config]=data_quaternion_noisy();
[n,~] =size(q_Robot_config);
n = round(n/2);
[HNRx1, HNtx1, HNRx2, HNtx2] =  hand_eye_calibration(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config, n);
disp(HNRx1);
disp(HNtx1);
disp(HNRx2);
disp(HNtx2);
HNnormR1 = norm(HNRx1 - Rx1);
HNnormP1 = norm(HNtx1 - tx1);
fprintf("The half set error measure for rotational part with axis-angle is: %.8f. \n", HNnormR1);
fprintf("The half set error measure for translational part with axis-angle is: %.8f \n", HNnormP1);
HNnormR2 = norm(HNRx2 - Rx2);
HNnormP2 = norm(HNtx2 - tx2);
fprintf("The half set error measure for rotational part with quaternion is: %.8f. \n", HNnormR2);
fprintf("The half set error measure for translational part with quaternion is: %.8f \n", HNnormP2);
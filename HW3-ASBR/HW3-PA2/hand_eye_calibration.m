function [Rx1, tx1, Rx2, tx2] =  hand_eye_calibration(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config, n)
M     =zeros(3,3);
A     =[];
b     =[];
tA    =[];
tB    =[];
for i =1:1:n-1
    %form kinematics matrix E-robot configuration 
    [REi]            = quat2rotm(q_Robot_config(i,:));
    [REiplus1]       = quat2rotm(q_Robot_config(i+1,:));
    tEi              = t_Robot_config(i,:);
    tEiplus1         = t_Robot_config(i+1,:);
    Ei               = [REi,   tEi';
                        0, 0, 0, 1];
    Eiplus1          = [REiplus1,   tEiplus1';
                        0, 0, 0,       1];
    % calculate transformation matrix Ai
    Ai               = Tinv(Ei) * Eiplus1;
    % calculate rotation matrix RAi and translation matrix tAi
    RAi              = Ai(1:3,1:3);
    tAi              = Ai(1:3,4);
    %calculate axis of Ai
    [alphai, ~]  = skewlog(RAi);
     % similar to the process of Ai calculation
    [RSi]            = quat2rotm(q_camera_config(i,:));
    [RSiplus1]       = quat2rotm(q_camera_config(i+1,:));
    tSi              = t_camera_config(i,:);
    tSiplus1         = t_camera_config(i+1,:);
    Si               = [RSi,   tSi';
                        0, 0, 0, 1];
    Siplus1          = [RSiplus1, tSiplus1';
                        0, 0, 0,       1];    
    Bi               = Si * Tinv(Siplus1);
    RBi              = Bi(1:3,1:3);
    tBi              = Bi(1:3,4);
    [betai, ~]  = skewlog(RBi);
    M                = M + betai*alphai';

    %form Ax=b equation
    A                = [A; 
                        eye(3,3)-RAi];
    tA               = [tA; tAi];
    tB               = [tB, tBi];
end

[Q,D]                  = eig(M'*M);
delta_inverse          = 1 ./ diag(sqrt(D));
MM_inverse             = Q*diag(delta_inverse)*inv(Q);
Rx1                    = MM_inverse*M';

% calculate tx
[~,m]                  = size(tB);
RxtB                   = [];
for i= 1:1:m    
    tBi                = tB(:, i);
    RxtBi              = Rx1*tBi;
    RxtB               = [RxtB; RxtBi];              
end

b                      = tA-RxtB;
tx1                    = A'*A\(A')*b;

%Quaternion analytical approach
M                      =[];
for i =1:1:n-1
   
    QEi                = q_Robot_config(i,:);
    QEiplus1           = q_Robot_config(i+1,:);
    QEi_star           = [QEi(1), -QEi(2:4)];
    QEi_inverse        = QEi_star/norm(QEi);
    QAi                = quatmultiply(QEi_inverse,QEiplus1)';

    QSi                = q_camera_config(i,:);
    QSiplus1           = q_camera_config(i+1,:);
    QSi_star           = [QSiplus1(1), -QSiplus1(2:4)];
    QSiplus1_inverse   = QSi_star/norm(QSiplus1);
    QBi                = quatmultiply(QSi,QSiplus1_inverse)';
    QBi                = QBi * (QBi(1)/abs(QBi(1)));

    M_qA_qBi           = generate_M(QAi,QBi);
    M                  = [M; M_qA_qBi];        
end

[~,~,V]                = svd(M);
V4                     = V(:,4);
Rx2                    = quat2rotm(V4');

[~,m]                  = size(tB);
RxtB                   = [];
for i= 1:1:m    
    tBi                = tB(:, i);
    RxtBi              = Rx2*tBi;
    RxtB               = [RxtB; RxtBi];              
end

b                      = tA-RxtB;
tx2                    = A'*A\(A')*b;
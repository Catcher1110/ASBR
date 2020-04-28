close all;
clear;
clc;
% Change file name here
test_name = 'a'; % it can be a-k
test_type = 'debug'; % it can be debug or unknown
calbodytxt = strcat('pa1-', test_type, '-', test_name, '-calbody.txt');
calreadingtxt = strcat('pa1-', test_type, '-', test_name, '-calreadings.txt');
empivottxt = strcat('pa1-', test_type, '-', test_name, '-empivot.txt');
optpivottxt = strcat('pa1-', test_type, '-', test_name, '-optpivot.txt');
%% 1
%import data of di and Di
datatable      = importdata(calbodytxt);
data_d         = datatable.data;
dj             = data_d(1:8,:);

step           = 43;
j              = 7;
datatable      = importdata(calreadingtxt);
data_D         = datatable.data;
Dj             = data_D(step*j+1:step*j+8,:);

[FD,RD,pd]           = computer_T(dj,Dj);

%% 2
%import data of ai and Ai
datatable      = importdata(calbodytxt);
data_a         = datatable.data;
aj             = data_a(9:16,:);

datatable      = importdata(calreadingtxt);
data_A         = datatable.data;
Aj             = data_A(9:16,:);

[FA,RA,pa]           = computer_T(aj,Aj);

%% 3
%import data of ci and compute Ci
datatable      = importdata(calreadingtxt);
data_C         = datatable.data;
Cj             = data_C(17:43,:);

datatable      = importdata(calbodytxt);
data_c         = datatable.data;
ci             = data_c(17:43,:);
ci             = [ci, ones(27,1)]';

Ci             = Tinv(FD) * FA * ci;
Ci             = Ci(:,1:3)';

%% 4
%import data of Gi
datatable      = importdata(empivottxt);
data_G         = datatable.data;
step           = 6;
% number of frame
A      =[];
b      =[];
G0 = data_G(1:6,:);
MeanG0 = mean(G0);
for j  = 0:11
Gj             = data_G(step*j+1:step*j+6,:);
gj             = Gj-MeanG0;
[FGj,Rj,pj]    = computer_T(gj,G0);
A              = [A;
                  Rj,-eye(3,3)];
b              = [b;-pj];   
end
x1              = A'*A\(A')*b;
Pdimple         = x1(4:6)';
disp(Pdimple);

%% 5
datatable      = importdata(optpivottxt);
data_DH        = datatable.data;
datatable      = importdata(calbodytxt);
data_d         = datatable.data;
dj             = data_d(1:8,:);

step           = 14;
A      =[];
b      =[];

D0             = data_DH(1:8,:);
[FD0,RD0,pd0]  = computer_T(dj,D0);
T0              = Tinv(FD0);
H0             = data_DH(9:14,:);
HT0            = T0 * [H0';ones(1,6)];
HT0            = HT0(1:3,:);
HT0            = HT0';
MeanHT0            = mean(HT0);
for j=0:11
% Get the tranformation     
Dj             = data_DH(step*j+1:step*j+8,:);
[FDj,~,~]  = computer_T(dj,Dj);
T              = Tinv(FDj);
% Read 
Hj             = data_DH(step*j+9:step*j+14,:);
HTj            = T * [Hj';ones(1,6)];
HTj            = HTj(1:3,:);
HTj            = HTj';
% HT0            = mean(HTj);
htj            = HTj-MeanHT0;          
[FHj,Rj,pj]    = computer_T(htj,HT0);
A              = [A;
                  Rj,-eye(3,3)];
b              = [b;-pj];  
end

x2              = A'*A\(A')*b;
Pdimple2         = x2(4:6)';
disp(Pdimple2);

%% Output expected C
datatable      = importdata(calbodytxt);
data_d         = datatable.data;
data_a         = datatable.data;
data_c         = datatable.data;
datatable      = importdata(calreadingtxt);
data_D         = datatable.data;
data_A         = datatable.data;
data_C         = datatable.data;

dj             = data_d(1:8,:);
ci             = data_c(17:43,:);
ci             = [ci, ones(27,1)]';
step           = 43;
Cexpected      = [];
for j = 0:7
    Dj             = data_D(step*j+1:step*j+8,:);
    [FD,~,~]       = computer_T(dj,Dj);
    aj             = data_a(9:16,:);
    Aj             = data_A(step*j+9:step*j+16,:);
    [FA,~,~]       = computer_T(aj,Aj);
    Cexpected      = [Cexpected, Tinv(FD) * FA * ci];
end
Cexpected = Cexpected(1:3, :)';
%%
filename = strcat(test_name, '-output.txt');
fid = fopen(filename,'w');
fprintf(fid, 'Our Output are: \n');
fprintf(fid, '%.2f, %.2f, %.2f\n', Pdimple(1,1), Pdimple(1,2), Pdimple(1,3));
fprintf(fid, '%.2f, %.2f, %.2f\n', Pdimple2(1,1), Pdimple2(1,2), Pdimple2(1,3));
[n, ~] = size(Cexpected);
for i = 1:1:n
    for j = 1:1:3
        if j ~= 3
            fprintf(fid, '%.2f, ', Cexpected(i,j));
        else
            fprintf(fid, '%.2f\n', Cexpected(i,j));
        end
    end
end
res = fclose(fid);

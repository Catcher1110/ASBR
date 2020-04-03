function varargout = PA2_Plot(varargin)
% PA2_PLOT MATLAB code for PA2_Plot.fig
%      PA2_PLOT, by itself, creates a new PA2_PLOT or raises the existing
%      singleton*.
%
%      H = PA2_PLOT returns the handle to a new PA2_PLOT or the handle to
%      the existing singleton*.
%
%      PA2_PLOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PA2_PLOT.M with the given input arguments.
%
%      PA2_PLOT('Property','Value',...) creates a new PA2_PLOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PA2_Plot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PA2_Plot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PA2_Plot

% Last Modified by GUIDE v2.5 02-Apr-2020 02:33:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PA2_Plot_OpeningFcn, ...
                   'gui_OutputFcn',  @PA2_Plot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before PA2_Plot is made visible.
function PA2_Plot_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PA2_Plot (see VARARGIN)

% Choose default command line output for PA2_Plot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PA2_Plot wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PA2_Plot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function theta1_Callback(hObject, eventdata, handles)
% hObject    handle to theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta1 as text
%        str2double(get(hObject,'String')) returns contents of theta1 as a double


% --- Executes during object creation, after setting all properties.
function theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta2_Callback(hObject, eventdata, handles)
% hObject    handle to theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta2 as text
%        str2double(get(hObject,'String')) returns contents of theta2 as a double


% --- Executes during object creation, after setting all properties.
function theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta3_Callback(hObject, eventdata, handles)
% hObject    handle to theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta3 as text
%        str2double(get(hObject,'String')) returns contents of theta3 as a double


% --- Executes during object creation, after setting all properties.
function theta3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta4_Callback(hObject, eventdata, handles)
% hObject    handle to theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta4 as text
%        str2double(get(hObject,'String')) returns contents of theta4 as a double


% --- Executes during object creation, after setting all properties.
function theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta5_Callback(hObject, eventdata, handles)
% hObject    handle to theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta5 as text
%        str2double(get(hObject,'String')) returns contents of theta5 as a double


% --- Executes during object creation, after setting all properties.
function theta5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta6_Callback(hObject, eventdata, handles)
% hObject    handle to theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta6 as text
%        str2double(get(hObject,'String')) returns contents of theta6 as a double


% --- Executes during object creation, after setting all properties.
function theta6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta7_Callback(hObject, eventdata, handles)
% hObject    handle to theta7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta7 as text
%        str2double(get(hObject,'String')) returns contents of theta7 as a double


% --- Executes during object creation, after setting all properties.
function theta7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T11_Callback(hObject, eventdata, handles)
% hObject    handle to T11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T11 as text
%        str2double(get(hObject,'String')) returns contents of T11 as a double


% --- Executes during object creation, after setting all properties.
function T11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T12_Callback(hObject, eventdata, handles)
% hObject    handle to T12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T12 as text
%        str2double(get(hObject,'String')) returns contents of T12 as a double


% --- Executes during object creation, after setting all properties.
function T12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T13_Callback(hObject, eventdata, handles)
% hObject    handle to T13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T13 as text
%        str2double(get(hObject,'String')) returns contents of T13 as a double


% --- Executes during object creation, after setting all properties.
function T13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T14_Callback(hObject, eventdata, handles)
% hObject    handle to T14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T14 as text
%        str2double(get(hObject,'String')) returns contents of T14 as a double


% --- Executes during object creation, after setting all properties.
function T14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T21_Callback(hObject, eventdata, handles)
% hObject    handle to T21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T21 as text
%        str2double(get(hObject,'String')) returns contents of T21 as a double


% --- Executes during object creation, after setting all properties.
function T21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T22_Callback(hObject, eventdata, handles)
% hObject    handle to T22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T22 as text
%        str2double(get(hObject,'String')) returns contents of T22 as a double


% --- Executes during object creation, after setting all properties.
function T22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_forward.
function btn_forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
t1 = str2double(handles.theta1.String);
t2 = str2double(handles.theta2.String);
t3 = str2double(handles.theta3.String);
t4 = str2double(handles.theta4.String);
t5 = str2double(handles.theta5.String);
t6 = str2double(handles.theta6.String);
t7 = str2double(handles.theta7.String);


k = 1000;
L1 = k * 0.333;
L2 = k * 0.316;
L3 = k * 0.0825;
L4 = k * 0.384;
L5 = k * 0.088;

L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', 0);
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2);
L(3) = Link('revolute', 'd', L2, 'a', 0, 'alpha', pi/2);
L(4) = Link('revolute', 'd', 0, 'a', L3, 'alpha', pi/2);
L(5) = Link('revolute', 'd', L4, 'a', -L3, 'alpha', -pi/2);
L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(7) = Link('revolute', 'd', 0, 'a', L5, 'alpha', pi/2);
Robot = SerialLink(L);
Robot.name = 'Franka Emika Panda';
Robot.plot([t1 t2 t3 t4 t5 t6 t7]);

%T = FK_space();

% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
T11 = str2double(handles.T11.String);
T12 = str2double(handles.T12.String);
T13 = str2double(handles.T13.String);
T14 = str2double(handles.T14.String);
T21 = str2double(handles.T21.String);
T22 = str2double(handles.T22.String);
T23 = str2double(handles.T23.String);
T24 = str2double(handles.T24.String);
T31 = str2double(handles.T31.String);
T32 = str2double(handles.T32.String);
T33 = str2double(handles.T33.String);
T34 = str2double(handles.T34.String);

Tsd = [T11, T12, T13, T14;
       T21, T22, T23, T24;
       T31, T32, T33, T34;
       0,   0,   0,   1];
   
t1 = str2double(handles.theta1.String);
t2 = str2double(handles.theta2.String);
t3 = str2double(handles.theta3.String);
t4 = str2double(handles.theta4.String);
t5 = str2double(handles.theta5.String);
t6 = str2double(handles.theta6.String);
t7 = str2double(handles.theta7.String);

theta0 = [t1 t2 t3 t4 t5 t6 t7];

k = 1000;
L1 = k * 0.333;
L2 = k * 0.316;
L3 = k * 0.0825;
L4 = k * 0.384;
L5 = k * 0.088;

L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', 0);
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2);
L(3) = Link('revolute', 'd', L2, 'a', 0, 'alpha', pi/2);
L(4) = Link('revolute', 'd', 0, 'a', L3, 'alpha', pi/2);
L(5) = Link('revolute', 'd', L4, 'a', -L3, 'alpha', -pi/2);
L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(7) = Link('revolute', 'd', 0, 'a', L5, 'alpha', pi/2);
Robot = SerialLink(L);
Robot.name = 'Franka Emika Panda';

% Define Fkb, Jb
M = [1 0 0 0.0880;
     0 -1 0 0;
     0 0 -1 0.9260;
     0 0 0 1];
wb = [0 0 -1; 0 -1 0; 0 0 -1; 0 1 0; 0 0 -1; 0 1 0; 0 0 1]';
qb = [-0.0880 0 0.5930; -0.0880 0 0.5930; -0.0880 0 0.2770; -0.0055 0 0.2770;
     -0.0880 0 -0.1070; -0.0880 0 -0.1070; 0 0 -0.1070]';
[~, num_Joint] = size(wb);
vb = zeros(3, num_Joint);
B = zeros(6, num_Joint);
for i = 1:1:num_Joint
    % Calculate the v_i = - cross(w_i, q_i)
    vb(:, i) = -cross(wb(:, i), qb(:, i));
    % Get the Screw Axis
    B(:, i) = [wb(:, i); vb(:, i)];
end
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
theta = [theta1 theta2 theta3 theta4 theta5 theta6 theta7];
FKb = FK_body(B, M, theta);
Jb = J_body(B, theta);

contents = cellstr(get(handles.menu_1, 'String'));
menuValue = contents{get(handles.menu_1, 'Value')};
disp(menuValue);
switch menuValue
    case '--- Choose Algorithm'
        [AllTheta, ~] = J_inverse_kinematics(FKb,Jb,Tsd,theta0);
    case 'Inverse Kinematics'
        [AllTheta, ~] = J_inverse_kinematics(FKb,Jb,Tsd,theta0);        
    case 'Transpose Kinematics'
        [AllTheta, ~] = J_transpose_kinematics(FKb,Jb,Tsd,theta0);
end

[n, ~] = size(AllTheta);
for i = 1:1:n
    thetai = AllTheta(i, :);
    handles.theta1.String = num2str(thetai(1));
    handles.theta2.String = num2str(thetai(2));
    handles.theta3.String = num2str(thetai(3));
    handles.theta4.String = num2str(thetai(4));
    handles.theta5.String = num2str(thetai(5));
    handles.theta6.String = num2str(thetai(6));
    handles.theta7.String = num2str(thetai(7));
    Robot.plot(thetai);
    pause(0.1);
end


function T23_Callback(hObject, eventdata, handles)
% hObject    handle to T23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T23 as text
%        str2double(get(hObject,'String')) returns contents of T23 as a double


% --- Executes during object creation, after setting all properties.
function T23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T24_Callback(hObject, eventdata, handles)
% hObject    handle to T24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T24 as text
%        str2double(get(hObject,'String')) returns contents of T24 as a double


% --- Executes during object creation, after setting all properties.
function T24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T31_Callback(hObject, eventdata, handles)
% hObject    handle to T31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T31 as text
%        str2double(get(hObject,'String')) returns contents of T31 as a double


% --- Executes during object creation, after setting all properties.
function T31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T32_Callback(hObject, eventdata, handles)
% hObject    handle to T32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T32 as text
%        str2double(get(hObject,'String')) returns contents of T32 as a double


% --- Executes during object creation, after setting all properties.
function T32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T33_Callback(hObject, eventdata, handles)
% hObject    handle to T33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T33 as text
%        str2double(get(hObject,'String')) returns contents of T33 as a double


% --- Executes during object creation, after setting all properties.
function T33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T34_Callback(hObject, eventdata, handles)
% hObject    handle to T34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T34 as text
%        str2double(get(hObject,'String')) returns contents of T34 as a double


% --- Executes during object creation, after setting all properties.
function T34_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T41_Callback(hObject, eventdata, handles)
% hObject    handle to T41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T41 as text
%        str2double(get(hObject,'String')) returns contents of T41 as a double


% --- Executes during object creation, after setting all properties.
function T41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T42_Callback(hObject, eventdata, handles)
% hObject    handle to T42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T42 as text
%        str2double(get(hObject,'String')) returns contents of T42 as a double


% --- Executes during object creation, after setting all properties.
function T42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T43_Callback(hObject, eventdata, handles)
% hObject    handle to T43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T43 as text
%        str2double(get(hObject,'String')) returns contents of T43 as a double


% --- Executes during object creation, after setting all properties.
function T43_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T44_Callback(hObject, eventdata, handles)
% hObject    handle to T44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T44 as text
%        str2double(get(hObject,'String')) returns contents of T44 as a double


% --- Executes during object creation, after setting all properties.
function T44_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in menu_1.
function menu_1_Callback(hObject, eventdata, handles)
% hObject    handle to menu_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns menu_1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from menu_1


% --- Executes during object creation, after setting all properties.
function menu_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to menu_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td1_Callback(hObject, eventdata, handles)
% hObject    handle to td1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td1 as text
%        str2double(get(hObject,'String')) returns contents of td1 as a double


% --- Executes during object creation, after setting all properties.
function td1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td2_Callback(hObject, eventdata, handles)
% hObject    handle to td2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td2 as text
%        str2double(get(hObject,'String')) returns contents of td2 as a double


% --- Executes during object creation, after setting all properties.
function td2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td3_Callback(hObject, eventdata, handles)
% hObject    handle to td3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td3 as text
%        str2double(get(hObject,'String')) returns contents of td3 as a double


% --- Executes during object creation, after setting all properties.
function td3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td4_Callback(hObject, eventdata, handles)
% hObject    handle to td4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td4 as text
%        str2double(get(hObject,'String')) returns contents of td4 as a double


% --- Executes during object creation, after setting all properties.
function td4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td5_Callback(hObject, eventdata, handles)
% hObject    handle to td5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td5 as text
%        str2double(get(hObject,'String')) returns contents of td5 as a double


% --- Executes during object creation, after setting all properties.
function td5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td6_Callback(hObject, eventdata, handles)
% hObject    handle to td6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td6 as text
%        str2double(get(hObject,'String')) returns contents of td6 as a double


% --- Executes during object creation, after setting all properties.
function td6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function td7_Callback(hObject, eventdata, handles)
% hObject    handle to td7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of td7 as text
%        str2double(get(hObject,'String')) returns contents of td7 as a double


% --- Executes during object creation, after setting all properties.
function td7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to td7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_calT.
function btn_calT_Callback(hObject, eventdata, handles)
% hObject    handle to btn_calT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
td1 = str2double(handles.td1.String);
td2 = str2double(handles.td2.String);
td3 = str2double(handles.td3.String);
td4 = str2double(handles.td4.String);
td5 = str2double(handles.td5.String);
td6 = str2double(handles.td6.String);
td7 = str2double(handles.td7.String);
thetad = [td1, td2, td3, td4, td5, td6, td7];


syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
theta = [theta1 theta2 theta3 theta4 theta5 theta6 theta7];
M = [1 0 0 0.0880;
     0 -1 0 0;
     0 0 -1 0.9260;
     0 0 0 1];
ws = [0 0 1; 0 1 0; 0 0 1; 0 -1 0; 0 0 1; 0 -1 0; 0 0 -1]';
qs = [0 0 0.3330; 0 0 0.3330; 0 0 0.6490; 0.0825 0 0.6490;
     0 0 1.0330; 0 0 1.0330; 0.0880 0 1.0330]';
[~, num_Joint] = size(ws);
vs = zeros(3, num_Joint);
S = zeros(6, num_Joint);
for i = 1:1:num_Joint
    % Calculate the v_i = - cross(w_i, q_i)
    vs(:, i) = -cross(ws(:, i), qs(:, i));
    % Get the Screw Axis
    S(:, i) = [ws(:, i); vs(:, i)];
end
FKs = FK_space(S, M, theta);

Tsd = double(subs(FKs, theta, thetad));
handles.T11.String = num2str(Tsd(1,1));
handles.T12.String = num2str(Tsd(1,2));
handles.T13.String = num2str(Tsd(1,3));
handles.T14.String = num2str(Tsd(1,4));
handles.T21.String = num2str(Tsd(2,1));
handles.T22.String = num2str(Tsd(2,2));
handles.T23.String = num2str(Tsd(2,3));
handles.T24.String = num2str(Tsd(2,4));
handles.T31.String = num2str(Tsd(3,1));
handles.T32.String = num2str(Tsd(3,2));
handles.T33.String = num2str(Tsd(3,3));
handles.T34.String = num2str(Tsd(3,4));

function varargout = advanced_teach(varargin)
% ADVANCED_TEACH MATLAB code for advanced_teach.fig
%      ADVANCED_TEACH, by itself, creates a new ADVANCED_TEACH or raises the existing
%      singleton*.
%
%      H = ADVANCED_TEACH returns the handle to a new ADVANCED_TEACH or the handle to
%      the existing singleton*.
%
%      ADVANCED_TEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ADVANCED_TEACH.M with the given input arguments.
%
%      ADVANCED_TEACH('Property','Value',...) creates a new ADVANCED_TEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before advanced_teach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to advanced_teach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help advanced_teach

% Last Modified by GUIDE v2.5 16-May-2022 12:22:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @advanced_teach_OpeningFcn, ...
    'gui_OutputFcn',  @advanced_teach_OutputFcn, ...
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

% --- Executes just before advanced_teach is made visible.
function advanced_teach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to advanced_teach (see VARARGIN)

% Choose default command line output for advanced_teach

handles.robot = varargin{1};
if nargin == 5
    handles.realBot = varargin{2};
    handles.usingRealBot = true;
else
    handles.usingRealBot = false;
end
handles.speedScale = 1;
handles.buttonDown = 0;
handles.eStop = 0;
set(handles.speed_slider, 'Min', 0);
set(handles.speed_slider, 'Max', 1);
set(handles.speed_slider, 'Value', 1);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes advanced_teach wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = advanced_teach_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure





% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% set(handles.pushbutton1,'String','off');
handles.buttonDown = 0;
guidata(hObject,handles);


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(handles.eStop == 0)
    handles.buttonDown = 1;
else
    handles.buttonDown = 0;
end

guidata(hObject,handles);


% --- Executes on button press in z_plus_button.
function z_plus_button_Callback(hObject, eventdata, handles)
% hObject    handle to z_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over z_plus_button.
function z_plus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to z_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
while handles.buttonDown
    %     if handles.usingRealBot == true
    %         handles.robot.model.animate(handles.realBot.current_joint_states.Position);
    %     end
    guidata(hObject,handles);
    %tr = handles.robot.model.fkine(handles.robot.model.getpos());
    %     newtr = transl(0,0,0.01) * tr;
    %     xdot = (newtr - tr)/0.05;
    xdot = [0,0,(1*handles.speedScale),0,0,0];
    newq = GUIRMRC(xdot, handles.robot);
    if handles.usingRealBot == true
        test = isalmost(handles.robot.model.getpos(),handles.realBot.current_joint_states.Position, 0.01);
        if all(test) == 0
            disp('Start of Traj does not match current robot position, please press reset button to reset sim');
            return
        end
        robotq = [handles.realBot.current_joint_states.Position; newq];
        handles.realBot.sendJointTrajectory(robotq);
    end
    handles.robot.model.animate(newq);
    drawnow();
    %pause(0.01);


    handles = guidata(hObject);

end



% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delete(hObject);




% --- Executes on button press in z_minus_button.
function z_minus_button_Callback(hObject, eventdata, handles)
% hObject    handle to z_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over z_minus_button.
function z_minus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to z_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown


    %     tr = handles.robot.model.fkine(handles.robot.model.getpos());
    %     newtr = transl(0,0,-0.01) * tr;
    %     xdot = (newtr - tr)/0.05;
    xdot = [0,0,(-1*handles.speedScale),0,0,0];
    newq = GUIRMRC(xdot, handles.robot);



    if handles.usingRealBot == true
        test = isalmost(handles.robot.model.getpos(),handles.realBot.current_joint_states.Position, 0.1);
        if all(test) == 0
            disp('Start of Traj does not match current robot position, please press reset button to reset sim');
            return
        end
        robotq = [handles.realBot.current_joint_states.Position; newq];
        handles.realBot.sendJointTrajectory(robotq);
    end
    handles.robot.model.animate(newq);
    drawnow();
    pause(0.01);


    handles = guidata(hObject);
end


% --- Executes on button press in y_plus_button.
function y_plus_button_Callback(hObject, eventdata, handles)
% hObject    handle to y_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in y_minus_button.
function y_minus_button_Callback(hObject, eventdata, handles)
% hObject    handle to y_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in x_plus_button.
function x_plus_button_Callback(hObject, eventdata, handles)
% hObject    handle to x_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in x_minus_button.
function x_minus_button_Callback(hObject, eventdata, handles)
% hObject    handle to x_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over x_plus_button.
function x_plus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to x_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



while handles.buttonDown

    %tr = handles.robot.model.fkine(handles.robot.model.getpos());
    %     newtr = transl(0,0,0.01) * tr;
    %     xdot = (newtr - tr)/0.05;
    xdot = [(1*handles.speedScale),0,0,0,0,0];
    newq = GUIRMRC(xdot, handles.robot);



    if handles.usingRealBot == true
        test = isalmost(handles.robot.model.getpos(),handles.realBot.current_joint_states.Position, 0.1);
        if all(test) == 0
            disp('Start of Traj does not match current robot position, please press reset button to reset sim');
            return
        end
        robotq = [handles.realBot.current_joint_states.Position; newq];
        handles.realBot.sendJointTrajectory(robotq);
    end
    handles.robot.model.animate(newq);
    drawnow();
    pause(0.01);


    handles = guidata(hObject);

end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over x_minus_button.
function x_minus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to x_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



while handles.buttonDown

    %tr = handles.robot.model.fkine(handles.robot.model.getpos());
    %     newtr = transl(0,0,0.01) * tr;
    %     xdot = (newtr - tr)/0.05;
    xdot = [(-1*handles.speedScale),0,0,0,0,0];
    newq = GUIRMRC(xdot, handles.robot);



    if handles.usingRealBot == true
        test = isalmost(handles.robot.model.getpos(),handles.realBot.current_joint_states.Position, 0.1);
        if all(test) == 0
            disp('Start of Traj does not match current robot position, please press reset button to reset sim');
            return
        end
        robotq = [handles.realBot.current_joint_states.Position; newq];
        handles.realBot.sendJointTrajectory(robotq);
    end
    handles.robot.model.animate(newq);
    drawnow();
    pause(0.01);


    handles = guidata(hObject);

end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over y_minus_button.
function y_minus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to y_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




while handles.buttonDown

    %tr = handles.robot.model.fkine(handles.robot.model.getpos());
    %     newtr = transl(0,0,0.01) * tr;
    %     xdot = (newtr - tr)/0.05;
    xdot = [0,(-1*handles.speedScale),0,0,0,0];
    newq = GUIRMRC(xdot, handles.robot);



    if handles.usingRealBot == true
        test = isalmost(handles.robot.model.getpos(),handles.realBot.current_joint_states.Position, 0.1);
        if all(test) == 0
            disp('Start of Traj does not match current robot position, please press reset button to reset sim');
            return
        end
        robotq = [handles.realBot.current_joint_states.Position; newq];
        handles.realBot.sendJointTrajectory(robotq);
    end
    handles.robot.model.animate(newq);
    drawnow();
    pause(0.01);


    handles = guidata(hObject);

end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over y_plus_button.
function y_plus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to y_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




while handles.buttonDown

    %tr = handles.robot.model.fkine(handles.robot.model.getpos());
    %     newtr = transl(0,0,0.01) * tr;
    %     xdot = (newtr - tr)/0.05;
    xdot = [0,(1*handles.speedScale),0,0,0,0];
    newq = GUIRMRC(xdot, handles.robot);



    if handles.usingRealBot == true
        test = isalmost(handles.robot.model.getpos(),handles.realBot.current_joint_states.Position, 0.1);
        if all(test) == 0
            disp('Start of Traj does not match current robot position, please press reset button to reset sim');
            return
        end
        robotq = [handles.realBot.current_joint_states.Position; newq];
        handles.realBot.sendJointTrajectory(robotq);
    end
    handles.robot.model.animate(newq);
    drawnow();
    pause(0.01);


    handles = guidata(hObject);



end


% --- Executes on button press in joint1_plus.
function joint1_plus_Callback(hObject, eventdata, handles)
% hObject    handle to joint1_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint2_plus.
function joint2_plus_Callback(hObject, eventdata, handles)
% hObject    handle to joint2_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint3_plus.
function joint3_plus_Callback(hObject, eventdata, handles)
% hObject    handle to joint3_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint4_plus.
function joint4_plus_Callback(hObject, eventdata, handles)
% hObject    handle to joint4_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint5_plus.
function joint5_plus_Callback(hObject, eventdata, handles)
% hObject    handle to joint5_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint6_plus.
function joint6_plus_Callback(hObject, eventdata, handles)
% hObject    handle to joint6_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint1_minus.
function joint1_minus_Callback(hObject, eventdata, handles)
% hObject    handle to joint1_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint2_minus.
function joint2_minus_Callback(hObject, eventdata, handles)
% hObject    handle to joint2_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint3_minus.
function joint3_minus_Callback(hObject, eventdata, handles)
% hObject    handle to joint3_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint4_minus.
function joint4_minus_Callback(hObject, eventdata, handles)
% hObject    handle to joint4_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint5_minus.
function joint5_minus_Callback(hObject, eventdata, handles)
% hObject    handle to joint5_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in joint6_minus.
function joint6_minus_Callback(hObject, eventdata, handles)
% hObject    handle to joint6_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint1_minus.
function joint1_minus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint1_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(1) = currentQ(1) - (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint2_minus.
function joint2_minus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint2_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(2) = currentQ(2) - (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint3_minus.
function joint3_minus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint3_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(3) = currentQ(3) - (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint4_minus.
function joint4_minus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint4_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(4) = currentQ(4) - (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint5_minus.
function joint5_minus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint5_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(5) = currentQ(5) - (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint6_minus.
function joint6_minus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint6_minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown
    currentQ = handles.robot.model.getpos();
    currentQ(6) = currentQ(6) - (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint1_plus.
function joint1_plus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint1_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown
    currentQ = handles.robot.model.getpos();
    currentQ(1) = currentQ(1) + (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint2_plus.
function joint2_plus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint2_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(2) = currentQ(2) + (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint3_plus.
function joint3_plus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint3_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(3) = currentQ(3) + (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint4_plus.
function joint4_plus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint4_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown
    currentQ = handles.robot.model.getpos();
    currentQ(4) = currentQ(4) + (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint5_plus.
function joint5_plus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint5_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown
    currentQ = handles.robot.model.getpos();
    currentQ(5) = currentQ(5) + (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over joint6_plus.
function joint6_plus_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to joint6_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

while handles.buttonDown

    currentQ = handles.robot.model.getpos();
    currentQ(6) = currentQ(6) + (0.02 * handles.speedScale);

    handles.robot.model.animate(currentQ);
    drawnow();
    handles = guidata(hObject);
end


% --- Executes on slider movement.
function speed_slider_Callback(hObject, eventdata, handles)
% hObject    handle to speed_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.speedScale = get(hObject, 'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function speed_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speed_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


if (handles.eStop == 1)
    handles.eStop = 0;
    handles.robot.eStopStatus = 0;
    guidata(hObject,handles);

end



% --- Executes on button press in e_stop_button.
function e_stop_button_Callback(hObject, eventdata, handles)
% hObject    handle to e_stop_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on start_button and none of its controls.
function start_button_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in e_stop.
function e_stop_Callback(hObject, eventdata, handles)
% hObject    handle to e_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of e_stop

if (get(hObject,'Value') == 1)
    disp("EMERGENCY STOP PRESSED");
    handles.eStop = get(hObject,'Value');
    handles.robot.eStopStatus = 1;
    guidata(hObject,handles);

end


% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
% hObject    handle to reset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.usingRealBot == true
    handles.robot.model.animate(handles.realBot.current_joint_states.Position);
end


% --- Executes on button press in open_gripper_button.
function open_gripper_button_Callback(hObject, eventdata, handles)
% hObject    handle to open_gripper_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.usingRealBot)
    handles.realBot.gripper.openGripper;
end
handles.robot.SetGripperState("gripperState", 0)


% --- Executes on button press in close_gripper_callback.
function close_gripper_callback_Callback(hObject, eventdata, handles)
% hObject    handle to close_gripper_callback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.usingRealBot)
    handles.realBot.gripper.closeGripper(600);
end
handles.robot.SetGripperState("gripperState", 1)


% --- Executes on button press in cable_function_callback.
function cable_function_callback_Callback(hObject, eventdata, handles)
% hObject    handle to cable_function_callback (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

PickAndPlaceEthCable(handles.robot)

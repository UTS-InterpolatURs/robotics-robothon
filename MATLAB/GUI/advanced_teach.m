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

% Last Modified by GUIDE v2.5 28-Apr-2022 19:46:34

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
handles.speedScale = 1;
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
global buttonDown;
buttonDown = 0;
% guidata(hObject,handles);


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global buttonDown;
buttonDown = 1;
disp('button down')
% guidata(hObject,handles);


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

global buttonDown;
while buttonDown

     %tr = handles.robot.model.fkine(handles.robot.model.getpos());
%     newtr = transl(0,0,0.01) * tr;
%     xdot = (newtr - tr)/0.05;
    xdot = [0,0,(1*handles.speedScale),0,0,0];
    q = handles.robot.model.getpos();
    J = handles.robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    newq = q + (0.01*qdot');
    J2 = handles.robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.01
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        break;
    end

    handles.robot.model.animate(newq);
    drawnow();
    pause(0.1);

  
    
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

global buttonDown;
while buttonDown

   
%     tr = handles.robot.model.fkine(handles.robot.model.getpos());
%     newtr = transl(0,0,-0.01) * tr;
%     xdot = (newtr - tr)/0.05;
    xdot = [0,0,(-1*handles.speedScale),0,0,0];
    q = handles.robot.model.getpos();
    J = handles.robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    
    newq = q + (0.01*qdot');
    J2 = handles.robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.01
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        break;
    end
    handles.robot.model.animate(newq);
    drawnow();
    pause(0.1);
disp('down');
    
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


global buttonDown;
while buttonDown

     %tr = handles.robot.model.fkine(handles.robot.model.getpos());
%     newtr = transl(0,0,0.01) * tr;
%     xdot = (newtr - tr)/0.05;
    xdot = [(1*handles.speedScale),0,0,0,0,0];
    q = handles.robot.model.getpos();
    J = handles.robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    newq = q + (0.01*qdot');
    J2 = handles.robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.01
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        break;
    end

    handles.robot.model.animate(newq);
    drawnow();
    pause(0.1);

  
    
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over x_minus_button.
function x_minus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to x_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global buttonDown;
while buttonDown

     %tr = handles.robot.model.fkine(handles.robot.model.getpos());
%     newtr = transl(0,0,0.01) * tr;
%     xdot = (newtr - tr)/0.05;
    xdot = [(-1*handles.speedScale),0,0,0,0,0];
    q = handles.robot.model.getpos();
    J = handles.robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    newq = q + (0.01*qdot');
    J2 = handles.robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.01
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        break;
    end

    handles.robot.model.animate(newq);
    drawnow();
    pause(0.1);

  
    
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over y_minus_button.
function y_minus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to y_minus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



global buttonDown;
while buttonDown

     %tr = handles.robot.model.fkine(handles.robot.model.getpos());
%     newtr = transl(0,0,0.01) * tr;
%     xdot = (newtr - tr)/0.05;
    xdot = [0,(-1*handles.speedScale),0,0,0,0];
    q = handles.robot.model.getpos();
    J = handles.robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    newq = q + (0.01*qdot');
    J2 = handles.robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.01
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        break;
    end

    handles.robot.model.animate(newq);
    drawnow();
    pause(0.1);

  
    
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over y_plus_button.
function y_plus_button_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to y_plus_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



global buttonDown;
while buttonDown

     %tr = handles.robot.model.fkine(handles.robot.model.getpos());
%     newtr = transl(0,0,0.01) * tr;
%     xdot = (newtr - tr)/0.05;
    xdot = [0,(1*handles.speedScale),0,0,0,0];
    q = handles.robot.model.getpos();
    J = handles.robot.model.jacob0(q);
    qdot = inv(J)*xdot';
    newq = q + (0.01*qdot');
    J2 = handles.robot.model.jacob0(newq);
    if sqrt(det(J2*J2')) < 0.01
        disp("Robot is close to singularity - Please use joint controls to move robot");
        disp("manipuability: ");
        disp(sqrt(det(J2*J2')));
        break;
    end

    handles.robot.model.animate(newq);
    drawnow();
    pause(0.1);

  
    
end

function varargout = sample(varargin)
% SAMPLE MATLAB code for sample.fig
%      SAMPLE, by itself, creates a new SAMPLE or raises the existing
%      singleton*.
%
%      H = SAMPLE returns the handle to a new SAMPLE or the handle to
%      the existing singleton*.
%
%      SAMPLE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SAMPLE.M with the given input arguments.
%
%      SAMPLE('Property','Value',...) creates a new SAMPLE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before sample_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to sample_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help sample

% Last Modified by GUIDE v2.5 25-Apr-2022 15:14:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @sample_OpeningFcn, ...
                   'gui_OutputFcn',  @sample_OutputFcn, ...
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


% --- Executes just before sample is made visible.
function sample_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sample (see VARARGIN)

% Choose default command line output for sample
handles.output = hObject;
handles.initial.Fs = 2e6; % 200KHz
handles.initial.fc = 20e3; % 200 Hz frequency of the pattern
handles.initial.phaseShift = 0; % phase shift in degree
handles.initial.cycles = 8;
handles.initial.voltAmp = 1; % Voltage Scale of pattern
handles.initial.Mu = 0.00605;     % deflection factor rad/volt
handles.initial.G = 567;     % deflector to powder bed distance in mm
handles.initial.wireDiam = 18e-3;
handles.initial.estimatedBeamDiam = 100e-3;
handles.initial.radSize = (tan(handles.initial.voltAmp * handles.initial.Mu) * handles.initial.G)/2; % radius of the pattern on powder bed
handles.initial.demo = 'off';

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes sample wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = sample_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% set(handles.text1,'String',num2str(str2num(get(handles.text1,'String'))+1))
% set(hObject,'String','off');



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.
function pushbutton1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if strcmpi(get(hObject,'String'),'off')
%     set(hObject,'String','on');
% end
global buttonDown;
while buttonDown

    
    if get(handles.fineAdj,'Value')
        set(handles.text1,'String',[num2str(str2num(strtok(get(handles.text1,'String'),' '))+.001,'%1.3f'),' mV'])
        pause(.05);
    else
        set(handles.text1,'String',[num2str(str2num(strtok(get(handles.text1,'String'),' '))+.05,'%1.3f'),' mV'])
        pause(eps);
    end
    if strcmpi(get(handles.text1,'String'),'nan mV')
        set(handles.text1,'String','0 mV');
    end

    TTi_Mediator(['SETUPCH 1;DCOFFS ',strtok(get(handles.text1,'String'),' ')]);
    
end



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
disp('test')
% guidata(hObject,handles);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton2.
function pushbutton2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global buttonDown;
while buttonDown

   
    disp('test');
    pause(0.1);
    
end


% --- Executes on button press in fineAdj.
function fineAdj_Callback(hObject, eventdata, handles)
% hObject    handle to fineAdj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of fineAdj


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton4.
function pushbutton4_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global buttonDown;
while buttonDown

   
    if get(handles.fineAdj,'Value')
         set(handles.text2,'String',[num2str(str2num(strtok(get(handles.text2,'String'),' '))-.001,'%1.3f'),' mV'])
        pause(.05);
    else
         set(handles.text2,'String',[num2str(str2num(strtok(get(handles.text2,'String'),' '))-.05,'%1.3f'),' mV'])
        pause(eps);
    end

    TTi_Mediator(['SETUPCH 2;DCOFFS ',strtok(get(handles.text2,'String'),' ')]);
    
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton3.
function pushbutton3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global buttonDown;
while buttonDown

   
    if get(handles.fineAdj,'Value')
         set(handles.text2,'String',[num2str(str2num(strtok(get(handles.text2,'String'),' '))+.001,'%1.3f'),' mV'])
        pause(.05);
    else
         set(handles.text2,'String',[num2str(str2num(strtok(get(handles.text2,'String'),' '))+.05,'%1.3f'),' mV'])
        pause(eps);
    end
    if strcmpi(get(handles.text2,'String'),'nan mV')
        set(handles.text2,'String','0 mV');
    end
    TTi_Mediator(['SETUPCH 2;DCOFFS ',strtok(get(handles.text2,'String'),' ')]);
    
end


% --- Executes on button press in arb_button.
function arb_button_Callback(hObject, eventdata, handles)
% hObject    handle to arb_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


if strcmpi(get(hObject,'String'),'Arm Devices')

    x = handles.initial.voltAmp * sin(2*pi*handles.initial.fc*...
        (0:1/handles.initial.Fs:handles.initial.cycles/handles.initial.fc) + handles.initial.phaseShift*(pi/180));
    y = handles.initial.voltAmp * cos(2*pi*handles.initial.fc*...
        (0:1/handles.initial.Fs:handles.initial.cycles/handles.initial.fc) + handles.initial.phaseShift*(pi/180));
    
      
%     figure
%     plot(x)
%     figure
%     plot(y)

    [waiHandle,stat] = TTi_ARB_GEN(x,y,handles.initial.voltAmp,handles.initial.Fs);

    if strcmpi(stat,'Failed')
            set(hObject,'Enable','on');
            errordlg('Failed to communicate with wave form generator','Communication Failure','replace');
            close(waiHandle);
            pause(.1);
            return;
    end

    %     PicoFs = [500e6,250e6,125e6,62.5e6,31.25e6,15.625e6,78.125e5,39.0625e5,19.53125e5,9.765625e5]; % picoscope sampling frequency (max 500MHz)
    try 
           handles.pico.handle = icdevice('PS3000a_IC_drv', ''); % Specify serial number as 2nd argument if required.
           connect(handles.pico.handle);

        % Connect device
    catch picoError
           errordlg([sprintf('Problem with connecting the devices\n'),...
               picoError.message],'Device is not connected');     
           set(hObject,'Enable','on');
           return;

    end


    if strcmpi(handles.pico.handle.Status,'closed')
               set(hObject,'Enable','on');
               return;
    end
    %        
    try
            waitbar(1,waiHandle,'All Done!');
            try
            close(waiHandle);
            pause(.1);
            catch
            end
    catch
    end

        set(hObject,'String','Disarm');
        set(hObject,'Enable','on');
        set(handles.start_button,'Enable','on');

else
    try
        delete(instrfind);
    catch
    end
    set(hObject,'String','Arm Devices');
    set(hObject,'Enable','on');
    set(handles.start_button,'Enable','off');
    set(handles.lock_button,'Enable','off');
end
guidata(hObject,handles);

% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

PicoFs = [500e6,250e6,125e6,62.5e6,31.25e6,15.625e6,78.125e5,39.0625e5...
    ,19.53125e5,9.765625e5]; % picoscope sampling frequency (max 500MHz)
handles.initial.radSize = (tan(handles.initial.Mu*handles.initial.voltAmp)*handles.initial.G)/2;

[fpass,fstop] = filter_estimator(handles.initial.fc,handles.initial.radSize*2,...
    handles.initial.wireDiam,handles.initial.estimatedBeamDiam);

%     set(handles.save_button,'Enable','on');
if strcmpi(get(handles.start_button,'String'),'Start') 
    set(handles.lock_button,'Enable','off');
    set(handles.measure_edit,'Enable','inactive');
    set(handles.start_button,'String','Stop');
    set(handles.start_button,'ForegroundColor','Red')
    set(handles.arb_button,'Enable','off');
    
    while strcmp(get(handles.start_button,'String'),'Stop')
        [handles.pico.signaldata,pico] = Pico_X2(handles.pico.handle,PicoFs(end-6)...
            ,handles.initial.fc,handles.initial.cycles,'non');
        
%         figure('Name','Test');
%         plot(handles.pico.signaldata.ch_a)
        
    [handles.minima] = Pic_Detection...
            (handles.pico.signaldata.ch_a,1,handles.initial.demo,fpass,fstop,PicoFs(end-6));
        
        [handles.deviation,handles.MisalignStatus] = MisalignmentAnalysis (handles.minima.global.index/PicoFs(end-6),...
            handles.initial.radSize); 
%         guidata(hObject,handles);
        if strcmpi(handles.MisalignStatus,'failed') || strcmpi(handles.MisalignStatus,'inadequate')
            if strcmpi(handles.MisalignStatus,'inadequate')
                errordlg(sprintf(['Inadequate data for misalignment analysis!',...
                '\nMinimum of 6 data points required!']),'Misalignment Analysis Failed!','replace');
%                 assignin('base','signaldata',handles.pico.signaldata);
            else
                errordlg(sprintf('No data for analysis!'),'Misalignment Analysis Failed!','replace');
            end
%             set(handles.start_button,'Enable','off');
            set(hObject,'String','Start');
            set(hObject,'ForegroundColor','Blue');
            set(handles.arb_button,'Enable','On');
            break;
        end
%         plotter(handles)
%         plot(handles.axes1,handles.deviation.dx + handles.CurrentProbe.X, ...
%             handles.deviation.dy + handles.CurrentProbe.Y,'Marker','o',...
%             'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerSize',8);
        pause(eps); % makes start button interruptable
        
        set(handles.measure_edit,'String',...
            sprintf(['dX =  ',num2str(handles.deviation.dx),'  mm','\ndY =  ',...
            num2str(handles.deviation.dy),'  mm','\nPattern Freq =  ',...
            num2str(handles.deviation.fc,'%0.1f'),'  Hz']));
        if strcmpi(handles.initial.demo,'demo')
            set(hObject,'String','Start');
            set(hObject,'ForegroundColor','Blue');
            set(handles.arb_button,'Enable','on');
        end
    end

else
    set(handles.start_button,'String','Start');
    set(handles.start_button,'ForegroundColor','Blue')
    set(handles.arb_button,'Enable','on');
    set(handles.lock_button,'Enable','on');
end
guidata(hObject,handles);



function measure_edit_Callback(hObject, eventdata, handles)
% hObject    handle to measure_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of measure_edit as text
%        str2double(get(hObject,'String')) returns contents of measure_edit as a double


% --- Executes during object creation, after setting all properties.
function measure_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to measure_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
   delete(instrfind)
catch
end
% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on button press in lock_button.
function lock_button_Callback(hObject, eventdata, handles)
% hObject    handle to lock_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if strcmpi(get(hObject,'String'),'Lock Beam') && strcmpi(get(handles.arb_button,'String'),'Disarm')
     set(hObject,'String','Done');
     set(handles.arb_button,'Enable','off');
     set(handles.start_button,'Enable','off');
     set(handles.pushbutton1,'Enable','off');
     set(handles.pushbutton2,'Enable','off');
     set(handles.pushbutton3,'Enable','off');
     set(handles.pushbutton4,'Enable','off');
     
%    Here I should implement an control algorithm, P should be enough but
%    PI can take care of the random error as well perhaps. Adjusting
%    parameters should be CH1 and CH2 DC offset that is locked previously.
PicoFs = [500e6,250e6,125e6,62.5e6,31.25e6,15.625e6,78.125e5,39.0625e5...
    ,19.53125e5,9.765625e5]; % picoscope sampling frequency (max 500MHz)
    
[fpass,fstop] = filter_estimator(handles.initial.fc,handles.initial.radSize,...
    handles.initial.wireDiam,handles.initial.estimatedBeamDiam);

    Kp(1:2) = [.1,.1];
    Ki(1:2) = [.0,.0];
    er(1:2)  = [0,0];
    erint(1:2) = [0,0];
    
     while strcmp(get(hObject,'String'),'Done')
        [handles.pico.signaldata,pico] = Pico_X2(handles.pico.handle,PicoFs(end-3)...
            ,handles.initial.fc,handles.initial.cycles,'non');
        [handles.minima] = Pic_Detection...
            (handles.pico.signaldata.ch_a,1,'middlesmooth',handles.initial.demo,fpass,fstop,PicoFs(end-3));
        
        [handles.deviation,handles.MisalignStatus] = MisalignmentAnalysis (handles.minima.global.index/PicoFs(end-3),...
            handles.initial.radSize); 
%         guidata(hObject,handles);
        if strcmpi(handles.MisalignStatus,'failed') || strcmpi(handles.MisalignStatus,'inadequate')
            if strcmpi(handles.MisalignStatus,'inadequate')
                errordlg(sprintf(['Inadequate data for misalignment analysis!',...
                '\nMinimum of 6 data points required!']),'Misalignment Analysis Failed!','replace');
            uiwait(gcf);
%                 assignin('base','signaldata',handles.pico.signaldata);
            else
                errordlg(sprintf('No data for analysis!'),'Misalignment Analysis Failed!','replace');
                uiwait(gcf);
            end
        end
        
          
        set(handles.measure_edit,'String',...
            sprintf(['dX =  ',num2str(handles.deviation.dx),'  mm','\ndY =  ',...
            num2str(handles.deviation.dy),'  mm','\nPattern Freq =  ',...
            num2str(handles.deviation.fc,'%0.1f'),'  Hz']));
        
        %% PI Algorithm
        
        MV = Kp .* er + Ki .* erint ;
        Ch_offset(1) = str2num(strtok(get(handles.text1,'String'),' '))+ (atan(MV(1)/handles.initial.G)/handles.initial.Mu);
        Ch_offset(2) = str2num(strtok(get(handles.text2,'String'),' '))+ (atan(MV(2)/handles.initial.G)/handles.initial.Mu);
%         disp(['SETUPCH 1;DCOFFS ',num2str(Ch1_offset,'%0.3f')])
        TTi_Mediator(['SETUPCH 1;DCOFFS ',num2str(Ch_offset(1),'%0.3f')]);
        set(handles.text1,'String',[num2str(Ch_offset(1),'%0.3f'),' mV']);
        pause(eps);
        TTi_Mediator(['SETUPCH 2;DCOFFS ',num2str(Ch_offset(2),'%0.3f')]);
        set(handles.text2,'String',[num2str(Ch_offset(2),'%0.3f'),' mV']);
        er(1) = 0 - handles.deviation.dx;
        er(2) = 0 - handles.deviation.dy;
        erint = (erint + er)/2;
        
        
%         assignin('base','ch',Ch_offset);
        
        
     end % end of while 
     
else  % if string is unlock
    set(hObject,'String','Lock Beam');
    set(handles.arb_button,'Enable','on');
    set(handles.start_button,'Enable','on');
    set(handles.pushbutton1,'Enable','inactive');
    set(handles.pushbutton2,'Enable','inactive');
    set(handles.pushbutton3,'Enable','inactive');
    set(handles.pushbutton4,'Enable','inactive');
     
end
    
    


% --- Executes on button press in setting_button.
function setting_button_Callback(hObject, eventdata, handles)
% hObject    handle to setting_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

txt = sprintf(['Fs = ',num2str(handles.initial.Fs),'\nfc = ',num2str(handles.initial.fc),...
   '\nphaseShift = ',num2str(handles.initial.phaseShift),'\ncycles = ',num2str(handles.initial.cycles),...
   '\nvoltAmp = ',num2str(handles.initial.voltAmp),'\nMu = ',num2str(handles.initial.Mu),...
   '\nG = ',num2str(handles.initial.G),'\nradSize = ',num2str(handles.initial.radSize),...
   '\ndemo = ',handles.initial.demo,'\nwireDiam = ',num2str(handles.initial.wireDiam),...
   '\nestimatedBeamDiam = ',num2str(handles.initial.estimatedBeamDiam)]);

prompt = sprintf('Change Settings Below.\nUnits are:\n(Hz, Degree, Volt, Rad/Vol, mm)');
resp = cell2mat(inputdlg(prompt,'Settings',11,mat2cell(txt)));
pause(eps);

if isempty(resp)
    warndlg('Default values will be used.','No Entry','replace');
    return;
end
if size(resp,1) ~= 11 && ~isempty(resp)
    errordlg(sprintf(['Number of input parameters do not agree.\n',...
        'Default values will be used']),'Invalid Entry');
    return;
end

for i = 1:11
    [param{i},val{i}.val] = strtok(regexprep(resp(i,:),' ? ?',''),'=');
    switch lower(param{i})
        case 'fs'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.Fs = str2num(val{i}.val(2:end));
            end
        case 'fc'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.fc = str2num(val{i}.val(2:end));
            end
        case 'phaseshift'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.phaseShift = str2num(val{i}.val(2:end));
            end
        case 'cycles'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.cycles = str2num(val{i}.val(2:end));
            end
        case 'voltamp'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.voltAmp = str2num(val{i}.val(2:end));
            end
        case 'mu'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.Mu = str2num(val{i}.val(2:end));
            end
        case 'g'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.G = str2num(val{i}.val(2:end));
            end
        case 'radsize'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.radSize = str2num(val{i}.val(2:end));
            end
        case 'demo'
            if strcmpi(val{i}.val(2:end),'on')
                handles.initial.demo = 'demo';
            else
                handles.initial.demo = 'off';
            end
        case 'wirediam'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.wireDiam = str2num(val{i}.val(2:end));
            end
        case 'estimatedbeamdiam'
            if ~isempty(str2num(val{i}.val(2:end)))
                handles.initial.estimatedBeamDiam = str2num(val{i}.val(2:end));
            end
            
        otherwise

            errordlg(sprintf(['Please check the parameters name and values.\nDefault values will be used.']),...
                'Unrecognized Entry','relace');
            
            handles.initial.Fs = 2e6; % 200KHz
            handles.initial.fc = 20e3; % 200 Hz frequency of the pattern
            handles.initial.phaseShift = 0; % phase shift in degree
            handles.initial.cycles = 8;
            handles.initial.voltAmp = 1; % Voltage Scale of pattern
            handles.initial.Mu = 0.00605;     % deflection factor rad/volt
            handles.initial.G = 567;     % deflector to powder bed distance in mm
            handles.initial.radSize = (tan(handles.initial.voltAmp * handles.initial.Mu) * handles.initial.G)/2; % radius of the pattern on powder bed
            handles.initial.demo = 'off';
            handles.initial.wireDiam = 18e-3;
            handles.initial.estimatedBeamDiam = 100e-3;
            uiwait(gcf);
            break;       
    end
end
      
msgbox(sprintf(['Following params in place\n\n','Fs = ',num2str(handles.initial.Fs),' Hz','\nfc = ',num2str(handles.initial.fc),' Hz',...
   '\nphaseShift = ',num2str(handles.initial.phaseShift),' Degree','\ncycles = ',num2str(handles.initial.cycles),...
   '\nvoltAmp = ',num2str(handles.initial.voltAmp),' Volt','\nMu = ',num2str(handles.initial.Mu),' Rad/Volt',...
   '\nG = ',num2str(handles.initial.G),' mm','\nradSize = ',num2str(handles.initial.radSize),' mm',...
   '\ndemo = ',handles.initial.demo,'\nwireDiam = ',num2str(handles.initial.wireDiam),' mm',...
   '\nestimatedBeamDiam = ',num2str(handles.initial.estimatedBeamDiam),' mm']),'New Parameters');
uiwait(gcf);
guidata(hObject,handles);

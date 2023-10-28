function varargout = posTracker(varargin)
% POSTRACKER M-file for posTracker.fig
%      POSTRACKER, by itself, creates a new POSTRACKER or raises the existing
%      singleton*.
%
%      H = POSTRACKER returns the handle to a new POSTRACKER or the handle to
%      the existing singleton*.
%
%      POSTRACKER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POSTRACKER.M with the given input arguments.
%
%      POSTRACKER('Property','Value',...) creates a new POSTRACKER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before posTracker_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed t6o posTracker_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help posTracker

% Last Modified by GUIDE v2.5 01-Apr-2022 09:13:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @posTracker_OpeningFcn, ...
                   'gui_OutputFcn',  @posTracker_OutputFcn, ...
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

% --- Executes just before posTracker is made visible.
function posTracker_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to posTracker (see VARARGIN)

% Choose default command line output for posTracker
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes posTracker wait for user response (see UIRESUME)
% uiwait(handles.figure1);
%init once 
handles=initOnce(handles);
guidata(hObject, handles);
%init camera
initCamera(handles);
%init Arduino
initArduino(handles);
%init data
handles=init(handles);
guidata(hObject, handles);

%detect cameras
function camDevices=detectCamera()
v=imaqhwinfo();
if isempty(v.InstalledAdaptors)
    camDevices=[];
    msgbox('No adaptor found, please install adaptors!',...
        'No Adaptor','warn');
    return;
end
%determine PC or MAC OS
if ispc
    hwInfo = imaqhwinfo('winvideo');
elseif isunix
    hwInfo = imaqhwinfo('macvideo');
end
if ~isempty(hwInfo.DeviceInfo)
    camDevices=hwInfo.DeviceInfo;
    
else
    camDevices=[];
    msgbox('No camera found! Running without camera mode!',...
        'NO Camera','warn');
end

%init functions
function initCamera(handles)
camDevices=detectCamera();
setappdata(0,'camDevices',camDevices);
%load cameras to camList listbox
camName=cell(1,length(camDevices));
for i=1:length(camDevices)
    camName{i}=camDevices(i). DeviceID;
end
set(handles.listbox_camList,'String',camName);
set(handles.listbox_camList,'Value',1);
if ~isempty(camDevices)
    vDefault='MJPG_640x480';            %'YUY2_320x240',
    camID=1;
    vFormats=camDevices(camID).SupportedFormats;
    vFnum=0;
    for i=1:length(vFormats)
        if strcmpi(vFormats{i},vDefault)
            vFnum=i;
        end
    end
    if vFnum>0
        vFmt=vDefault;
    else
        vFmt=camDevices(camID).DefaultFormat;
        vFnum=1;
    end
    %load all supported Formats to listbox
    set(handles.listbox_vFormats,'String',vFormats);
    set(handles.listbox_vFormats,'Value',vFnum);    
    
    vidobj = createVidObj(camDevices(camID),vFmt,handles);
    %init the camera-panel information
    initCamPanel(vidobj,handles);
else
    vidobj=[];
end
setappdata(0,'vidobj',vidobj);

%init the Arduino 
function initArduino(handles)
setappdata(0,'myArduino',[]);
port=instrhwinfo ('serial');
spNum=length(port.SerialPorts);
if spNum>0
    %load the list
    set(handles.listbox_Arduino_portNames,'String',port.SerialPorts);
    if spNum>0
        set(handles.listbox_Arduino_portNames,'Value',1);  
        portName=port.SerialPorts{1};
        setappdata(0,'serialPort',portName);
%         try 
%             myArduino=serial(portName,'BaudRate',9600);
%             %set(myArduino,'Timeout',1);    
%             setappdata(0,'myArduino',myArduino);
%             fopen(myArduino);
%             disp('Connect to Arduino, and initialize...');
%             %pause(2);    
%             %sendArduino(myArduino,'C','0','A');
%             disp('Arduino connected successfully!');
%         catch exception
%             msgbox('No Arduino found, please check serial port and try again!',...
%                 'NO Arduino','warn');
%             setappdata(0,'myArduino',[]);
%             return;
%         end
    end
else
    msgbox('No serial port found!','No Port','warn');
    return;
end


%init only once
function handles=initOnce(handles)
versionName='posTracker1.97-2022';
set(handles.figure1,'Name',versionName);
setappdata(0,'versionName',versionName);
setappdata(0,'PathName','D:\');
set(handles.text_folderName,'String','D:\');
setappdata(0,'recTime',1800);      %total record time
set(handles.edit_RecTime,'String',s2hhmmss(1800,0));
threshold=100;          %signal/noise ratio to detect lick/flash
set(handles.edit_Threshold,'String',num2str(threshold));
setappdata(0,'threshold',threshold);
setappdata(0,'VideoSaveTag',1);
set(handles.checkbox_VideosaveTag,'value',1);
setappdata(0,'OnlineTag',1);
set(handles.checkbox_OnlineTag,'value',1);
setappdata(0,'taskTag',0);  %0=default,1=NOR,2=CFC
setappdata(0,'detectFreezingTag',0);
set(handles.checkbox_detectFreezing,'Value',0);
freezingPars=[10,1.0];          %[v-th, duration-cutoff];
setappdata(0,'freezingPars',freezingPars);
set(handles.edit_freezingVth','String',num2str(freezingPars(1)));
set(handles.edit_freezingCutoff','String',num2str(freezingPars(2)));
setappdata(0,'replayTag',0);                %0=no-replay/1=normal replay/2=pause/3=forward/4=backward
setappdata(0,'plotLocFlag',false);
set(handles.checkbox_replay,'value',0);
setappdata(0,'frameStep',2);        %process data every framestep 
% camera panel
handles.camZoom=0;
set(handles.slider_Camera_Zoom,'Min',0','Max',10,'sliderstep',[0.1,0.1]);
set(handles.slider_Camera_Zoom,'value',handles.camZoom);
set(handles.edit_Camera_Zoom,'String',num2str(handles.camZoom));
handles.camPan=0;
set(handles.slider_Camera_Pan,'Min',-50','Max',50,'sliderstep',[0.01,0.01]);
set(handles.slider_Camera_Pan,'value',handles.camPan);
set(handles.edit_Camera_Pan,'String',num2str(handles.camPan));
handles.camTilt=0;
set(handles.slider_Camera_Tilt,'Min',-50','Max',50,'sliderstep',[0.01,0.01]);
set(handles.slider_Camera_Tilt,'value',handles.camTilt);
set(handles.edit_Camera_Tilt,'String',num2str(handles.camTilt));
%Arduino panel
ardInfo=struct;
ardInfo.totalTime=600;      %total record time (sec)
ardInfo.delay=120;       %delay time (sec) to start outputs
ardInfo.repeat=3;       
ardInfo.interval=90;       %interval time (sec) between outputs
ardInfo.toneFlag=1;
ardInfo.toneDuration=30;
ardInfo.shockFlag=1;
ardInfo.shockDuration=2;
updateProtPanel(ardInfo,handles);
ardInfo.laserFlag=0;
set(handles.checkbox_Ardu_Output_Laser,'Value',ardInfo.laserFlag);
conditionedArea=1;
setappdata(0,'conditionedArea',conditionedArea);
set(handles.edit_conditionedAreaNum,'String',num2str(conditionedArea));
setappdata(0,'ardInfo',ardInfo);


function handles=init(handles)        
setappdata(0,'ctrlState',0);            %control-state: 0/none;1/preview;2/record;3/load;4/analyze
setappdata(0,'ctrlRecordTag',0);        %ctrl-record Tag:1/record;0/stop
setappdata(0,'ctrlPreviewTag',0);       %ctrl-preview Tag:1/preview;0/stop
setappdata(0,'ctrlROIShowTag',0);
ROI=struct('x',[],'y',[],'width',[],'height',[],'handle',[]);
setappdata(0,'ROI',ROI);
setappdata(0,'nowFrame',[]);        %current frame (displayed) image
setappdata(0,'aviobj',[]);
setappdata(0,'totalTim',0);
setappdata(0,'chamberMap',[]);      %map of chambers
setappdata(0,'hfig',[]);
setappdata(0,'hreplayer',[]);
setappdata(0,'hfreezing',[]);       %figure handle to plotFreezing
setappdata(0,'bkImgBuffer',uint8([]));
setappdata(0,'bkImg2D',uint8([]));
setappdata(0,'nextArdTime',[inf,0]);              %next [time,number] to send command to Arduino
initData();

%init the data
function initData()
setappdata(0,'nowTim',[0,0]);       %saved timepoints in frames_afteracquired and Timer
setappdata(0,'trackData',[]);       %track data = [time,x,y];
setappdata(0,'velocityData',[]);    %[time,speed]
setappdata(0,'moveTag',[0,0]);         %[time,activityState] 1=walk,0=rest over 1s(sampling period)
setappdata(0,'scoreData',[]);
setappdata(0,'binData',[]);
setappdata(0,'laserChamberTim',0);         %entering time in photo-chamber
setappdata(0,'freezingData',zeros(1,3));       %[start,end,duration] in sec for each freezing
setappdata(0,'freezingTotal',0);          %total freezing time in second
global prePosHandle;
prePosHandle=[];
%init the hardware
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)  
    %laser off
    fwrite(myArduino,'0','uchar');
end


%create video object
function vidobj=createVidObj(camDevice,vFormat,handles)
if ispc
    vidobj = videoinput('winvideo',camDevice.DeviceID,vFormat);
elseif isunix
    vidobj = videoinput('macvideo',camDevice.DeviceID,vFormat);
end
%set(vidobj,'ReturnedColorSpace','rgb');
set(vidobj,'ReturnedColorSpace','grayscale');
%init important functions here
set(vidobj,'StartFcn',{@myStartCam,handles});
set(vidobj,'StopFcn',{@myStopCam,handles});
set(vidobj,'TimerPeriod',0.3);
set(vidobj,'TimerFcn',{@myTimerCam,handles});
set(vidobj,'FramesPerTrigger',inf);
set(vidobj,'TriggerRepeat',0); 
set(vidobj,'FramesAcquiredFcnCount',10);
set(vidobj,'FramesAcquiredFcn',{@myFrames_AfterAcquired,handles});
% src = getselectedsource(vidobj);   
% srcInfo=get(src);
% if isfield(srcInfo,'ExposureMode')
%        set(src,'ExposureMode','manual');
%        src.Exposure=-7;
%        src.Brightness=150;
% end
% if isfield(srcInfo,'FocusMode')
%        set(src,'FocusMode','manual');
%        src.Focus=4;
% end
% if isfield(srcInfo,'FrameRate')
%        src.FrameRate='30';
% end
% if isfield(srcInfo,'WhiteBalance')
%          set(src,'WhiteBalanceMode','manual');
%          src.WhiteBalance = 4200;
% end

    
%init the camera pannel 
function initCamPanel(vidobj,handles)
src = getselectedsource(vidobj);   
srcInfo=get(src);
if isfield(srcInfo,'Zoom')
        src.Zoom = 1;
        set(handles.edit_Camera_Zoom,'Enable','on');
        set(handles.slider_Camera_Zoom,'Enable','on');    
else
        set(handles.edit_Camera_Zoom,'Enable','off');
        set(handles.slider_Camera_Zoom,'Enable','off');
end
if isfield(srcInfo,'Tilt')
        src.Tilt = 0;
        set(handles.edit_Camera_Tilt,'Enable','on');
        set(handles.slider_Camera_Tilt,'Enable','on');    
else
        set(handles.edit_Camera_Tilt,'Enable','off');
        set(handles.slider_Camera_Tilt,'Enable','off');
end
if isfield(srcInfo,'Pan')
        src.Pan = 0;
        set(handles.edit_Camera_Tilt,'Enable','on');
        set(handles.slider_Camera_Tilt,'Enable','on');    
else
        set(handles.edit_Camera_Pan,'Enable','off');
        set(handles.slider_Camera_Pan,'Enable','off');
end


function updateProtPanel(ardInfo,handles)
set(handles.edit_Ardu_delay,'String',ardInfo.delay);  
set(handles.edit_Ardu_repeat,'String',ardInfo.repeat);
set(handles.edit_Ardu_interval,'String',ardInfo.interval);
set(handles.checkbox_Ardu_Output_Tone,'Value',ardInfo.toneFlag);
set(handles.edit_Ardu_Output_ToneDuration,'String',ardInfo.toneDuration);
set(handles.checkbox_Ardu_Output_Shock,'Value',ardInfo.shockFlag);
set(handles.edit_Ardu_Output_ShockDuration,'String',ardInfo.shockDuration);


% --- Outputs from this function are returned to the command line.
function varargout = posTracker_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in ctrl_Preview.
function ctrl_Preview_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Preview (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vidobj=getappdata(0,'vidobj');
ctrlPreviewTag=getappdata(0,'ctrlPreviewTag');
if ctrlPreviewTag==1
    closepreview(vidobj);
    set(hObject,'String','Preview');
    ctrlPreviewTag=0;
    ctrlState=0;        
    %F=getframe;
    %nowFrame=F.cdata;
else        
    set(hObject,'String','Stop');
    ctrlPreviewTag=1;
    ctrlState=1;    
    preview(vidobj);
end
setappdata(0,'ctrlState',ctrlState);
setappdata(0,'ctrlPreviewTag',ctrlPreviewTag);
guidata(hObject,handles);

        
% --- Executes on button press in ctrl_Record.
function ctrl_Record_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Record (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
VideoSaveTag=getappdata(0,'VideoSaveTag');
vidobj=getappdata(0,'vidobj');
frameNums=getappdata(0,'frameNums');
recTime=getappdata(0,'recTime');
ctrlRecordTag=getappdata(0,'ctrlRecordTag');
src = getselectedsource(vidobj);
srcInfo = get(src);
if ctrlRecordTag==1   
    stop(vidobj);
else        
    set(hObject,'String','Stop');
    OnlineTag=getappdata(0,'OnlineTag');
    ROI=getappdata(0,'ROI');
    bkImg2D=getappdata(0,'bkImg2D');
    if isempty(bkImg2D) 
        if OnlineTag
            %msgbox('No background image, On-line processing off!','Warning','warn');
            set(handles.checkbox_OnlineTag,'value',0);
            setappdata(0,'OnlineTag',0);
        end
    else
        %clear the figure
        showBackground_ROI(bkImg2D,ROI,handles);
    end
   
    frameNums(1:2)=[0,0];
    if isfield(srcInfo,'FrameRate')
        frameNums(3)=str2double(src.FrameRate);
    else
        frameNums(3)=30;
    end
    setappdata(0,'frameNums',frameNums);
    setappdata(0,'ctrlRecordTag',1);
    setappdata(0,'ctrlState',2);
    ardInfo=getappdata(0,'ardInfo');
    setappdata(0,'nextArdTime',[ardInfo.delay,1]);
    if recTime==0
        set(vidobj, 'FramesPerTrigger', inf);
    else
        set(vidobj, 'FramesPerTrigger', recTime*30);
    end
    if VideoSaveTag
        set(vidobj, 'LoggingMode', 'Disk&memory');
        FileName=getFileName('.avi');
        setappdata(0,'FileName',FileName);
        PathName=getappdata(0,'PathName');
        fname=strcat(PathName,FileName);
        aviobj=VideoWriter(fname);
        vidobj.DiskLogger = aviobj;
    else
        set(vidobj, 'LoggingMode', 'memory');
    end
    
    initData();
    start(vidobj);    
end
guidata(hObject,handles);

%To show the background Image and ROI
function showBackground_ROI(backgroundImg,ROI,handles)
[vHei,vWid,chs]=size(backgroundImg);
hfig=getappdata(0,'hfig');
hfig=creatFigure(hfig,vWid,vHei);
setappdata(0,'hfig',hfig);
imagesc(backgroundImg);
colormap('gray');
axis image off;
hold on;
%draw the ROI
n=length(ROI.x);
if n>0
    cls=get(gca,'colororder');
	for i=1:n
        h=rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
                'LineStyle','--','edgecolor',cls(i+1,:));
        ROI.handle(i)=h;    
    end
    setappdata(0,'ROI',ROI);
end    

%get handle to figure
function handle=creatFigure(hfig,vWid,vHei)
if ishandle(hfig)
	handle=figure(hfig);
    cla;
else       
    handle=figure('position',[10,10,vWid,vHei]);  
	set(gca,'position',[0,0,1,1]);
	axis image off;
end


%save the data to .mat
function saveMatFile()
FileName=getappdata(0,'FileName'); 
PathName=getappdata(0,'PathName');
ROI=getappdata(0,'ROI');
threshold=getappdata(0,'threshold');
trackData=getappdata(0,'trackData');
velocityData=getappdata(0,'velocityData');   
bkImg2D=getappdata(0,'bkImg2D');
chamberMap=getappdata(0,'chamberMap');
scoreData=getappdata(0,'scoreData');
freezingData=getappdata(0,'freezingData');       
freezingTotal=getappdata(0,'freezingTotal');  
freezingPars=getappdata(0,'freezingPars');
moveTag=getappdata(0,'moveTag');
ardInfo=getappdata(0,'ardInfo');
mfile=strcat(FileName(1:length(FileName)-4),'.mat');
filename=fullfile(PathName,mfile);
save(filename,'filename','ROI','trackData','bkImg2D','chamberMap',...
    'scoreData','velocityData','freezingData','freezingTotal',...
    'moveTag','ardInfo','freezingPars','threshold');

%function when start(vidobj)
function myStartCam(hObject, eventdata,handles)
setappdata(0,'nowTim',[0,0]);  
tic;

%function when stop(vidobk)
function myStopCam(hObject, eventdata,handles)
vidobj=getappdata(0,'vidobj');    
%stoppreview(vidobj);
set(handles.ctrl_Record,'String','Record');
setappdata(0,'ctrlState',0);
setappdata(0,'ctrlRecordTag',0);
setappdata(0,'ctrlState',4);
OnlineTag=getappdata(0,'OnlineTag');
ROI=getappdata(0,'ROI');
if OnlineTag    
    trackData=getappdata(0,'trackData');
    bkImg=getappdata(0,'bkImg2D');
    chamberMap=getappdata(0,'chamberMap');
    %remove zeros(mouse not found)
    idx=find(trackData(:,2)>0 | trackData(:,3)>0);
    trackData=trackData(idx,:);
    setappdata(0,'trackData',trackData);
    if ~isempty(chamberMap)
        %calculate the index (time spent in chambers)
        [scoreData,binData,posHotMap]=getScore(chamberMap,trackData);
        setappdata(0,'scoreData',scoreData);
        setappdata(0,'binData',binData);
        disp(scoreData);
    end
    %draw the mouse track
    FileName=getappdata(0,'FileName'); 
    PathName=getappdata(0,'PathName');
    fn=fullfile(PathName,FileName);

    showTrack(trackData,scoreData,bkImg,ROI,fn,handles);
    
    detectFreezingTag=getappdata(0,'detectFreezingTag');
    freezingData=getappdata(0,'freezingData');   
    if detectFreezingTag && ~isempty(freezingData)
        %if no end time for last episode
        if freezingData(end,1)>0 && freezingData(end,2)==0
            nowTim=getappdata(0,'nowTim');
            freezingData(end,2)=nowTim(2);
            freezingData(end,3)=freezingData(end,2)-freezingData(end,1);
        end
        %remove very short episodes (noise)
        idx=find(freezingData(:,3)>=1.0);
        freezingData=freezingData(idx,:);
        freezingTotal=sum(freezingData(:,3));
        set(handles.text_freezingSec,'String',num2str(freezingTotal,'%8.1f'));
        setappdata(0,'freezingTotal',freezingTotal);  
        setappdata(0,'freezingData',freezingData);  
    end
    
    %draw the hotmap of position(for long-term recording)
    %showPosHotMap(posHotMap,scoreData,ROI,handles);
    if ~isempty(binData)
        showbinData(binData,handles);
    end
    %save the data to txt-file
    fn=strcat(FileName(1:end-4),'.txt');
    fn=strcat(PathName,fn);
    saveTxtData(fn);
	%also save the data to .mat	
    saveMatFile(); 
end

%save the video
VideoSaveTag=getappdata(0,'VideoSaveTag');
if VideoSaveTag   
    wait(vidobj,5);
end



%function when timer(for video)
function myTimerCam(hObject, eventdata,handles)
%update timer
nTim=toc;
set(handles.txt_Timer,'String',s2hhmmss(nTim,0));
nowTim=getappdata(0,'nowTim');
%real-time freezing detection
OnlineTag=getappdata(0,'OnlineTag');
detectFreezingTag=getappdata(0,'detectFreezingTag');
if OnlineTag && detectFreezingTag && nTim>=1
    c0th=0.1;        %threshold for sleep/freezing judge, default[0.05,0.2] for non-cable; use bigger for cable (noisy) 
    binT=1;        %time window to calculate freezing time
    freezingData=getappdata(0,'freezingData');     
    freezingTotal=getappdata(0,'freezingTotal');    
    frznum=size(freezingData,1);
    moveTag=getappdata(0,'moveTag');
    idx=find(moveTag(:,1)>=nTim-binT & moveTag(:,1)<nTim);  %per second
    c0=mean(moveTag(idx,2),1);
    if c0<c0th
        freezingTotal=freezingTotal+nTim-nowTim(2);
        setappdata(0,'freezingTotal',freezingTotal);   
        if freezingData(frznum,1)==0
            freezingData(frznum,1)=nTim;
        end
    else
        if freezingData(frznum,1)>0
            freezingData(frznum,2)=nTim;
            freezingData(frznum,3)=freezingData(frznum,2)-freezingData(frznum,1);
            freezingData(frznum+1,:)=zeros(1,3);
        end
    end
    set(handles.text_freezingSec,'String',num2str(freezingTotal,'%8.1f'));
    setappdata(0,'freezingData',freezingData);   
end
%control hardwares via arduino
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    %trackData=getappdata(0,'trackData');
    ardInfo=getappdata(0,'ardInfo');
    nextArdTime=getappdata(0,'nextArdTime');
    if nTim>=nextArdTime(1) && nextArdTime(2)<=ardInfo.repeat
        if ardInfo.toneFlag
        	fwrite(myArduino,'T','uchar'); 
        end
        if ardInfo.shockFlag
            fwrite(myArduino,'S','uchar'); 
        end
        nextArdTime(1)=nextArdTime(1)+ardInfo.toneDuration+ardInfo.interval;
        nextArdTime(2)=nextArdTime(2)+1;
        setappdata(0,'nextArdTime',nextArdTime);
    end
    if ardInfo.laserFlag
        laser2chamber(nTim);
    end
end
setappdata(0,'nowTim',[nowTim(1),nTim]);

%laser stimulation for 2-chamber place conditioning
function laser2chamber(nowTim)
trackData=getappdata(0,'trackData');
chamberMap=getappdata(0,'chamberMap');
conditionedArea=getappdata(0,'conditionedArea'); 
laserChamberTim=getappdata(0,'laserChamberTim');
x=0;y=0;
if ~isempty(trackData)
	x=round(trackData(end,2));
	y=round(trackData(end,3));
end
if nowTim-laserChamberTim<=5
        if x>0 && y>0
                if chamberMap(y,x)==conditionedArea
                    fwrite(myArduino,'X','uchar');
                else
                    fwrite(myArduino,'0','uchar');
                    setappdata(0,'laserChamberTim',toc);
                end
        end
elseif nowTim-laserChamberTim>5 && nowTim-laserChamberTim<=5+5
        fwrite(myArduino,'0','uchar');
        if x>0 && y>0
                if chamberMap(y,x)~=conditionedArea
                    setappdata(0,'laserChamberTim',toc);
                end
        end
elseif nowTim-laserChamberTim>5+5        %stop 5s after 20s contineous stimulation
        setappdata(0,'laserChamberTim',toc);
end

%function when certain frames acquired
function myFrames_AfterAcquired(hObject, eventdata,handles) 
OnlineTag=getappdata(0,'OnlineTag');
if OnlineTag
    vidobj=getappdata(0,'vidobj');
    nFrames=vidobj.FramesAvailable;
    if nFrames>0
        nTim=toc;
        nowTim=getappdata(0,'nowTim');
        frameNums=getappdata(0,'frameNums');
        trackData=getappdata(0,'trackData');
        velData=getappdata(0,'velocityData');
        bkImg=getappdata(0,'bkImg2D');
        th=getappdata(0,'threshold');
        if ~isempty(trackData)
            preTrkDat=trackData(end,:);
        else
            preTrkDat=[];
        end
        vidData=peekdata(vidobj,nFrames);
        flushdata(vidobj);
        nowFrmIdx=frameNums(1)+(1:nFrames);
        %get trackData
        trackData(nowFrmIdx,2:3)=getPosData1(vidData,bkImg,th);
        %trackData(nowFrmIdx,1)=nowFrmIdx/frameNums(3);
        trackData(nowFrmIdx,1)=linspace(nTim-nFrames/frameNums(3),nTim,nFrames);
        frameNums(1)=frameNums(1)+nFrames;
        frameNums(2)=frameNums(1); 
        setappdata(0,'trackData',trackData);
        setappdata(0,'frameNums',frameNums);
        %calculate the velocity
        velData(nowFrmIdx,1:2)=getVelocity(trackData(nowFrmIdx,:),preTrkDat);
        moveTag=getappdata(0,'moveTag');
        midx=size(moveTag,1)+1;
        moveTag(midx,1)=velData(nowFrmIdx(1),1);
        moveTag(midx,2)=0;
        mv=mean(velData(nowFrmIdx,2),1);
        vd=sqrt((trackData(nowFrmIdx(end),2)-trackData(nowFrmIdx(1),2)).^2+...
            (trackData(nowFrmIdx(end),3)-trackData(nowFrmIdx(1),3)).^2)./(nTim-nowTim(1));
        %important for sleep/freezing-detection!
        if mv>15 && vd>mv*0.15
            moveTag(midx,2)=1;
        end
        setappdata(0,'moveTag',moveTag);
        
        %show realtime figure
        ShowRunData(trackData(nowFrmIdx,2:3),handles);  
        %update the real capture framerate
        rlRate=min(nFrames/(nTim-nowTim(1)),frameNums(3));            %real-time change
        set(handles.cam_frmHz_Indicator,'Position',[12,83.2,15,50*rlRate/frameNums(3)]);
        set(handles.cam_frmHz_Indicator,'BackgroundColor',[round(1-rlRate/frameNums(3)+0.2),...
                round(rlRate/frameNums(3)-0.2),1-rlRate/frameNums(3)]);
        setappdata(0,'nowTim',[nTim,nowTim(2)]);
        setappdata(0,'velocityData',velData);
        clear vidData;
    end
end

function velData=getVelocity(trackData,preTrkDat)
[frmNum,b]=size(trackData);
velData=zeros(frmNum,2);
velData(:,1)=trackData(:,1);
t=mean(trackData(2:end,1)-trackData(1:end-1,1));
if isempty(preTrkDat)
    preTrkDat=trackData(1,:);
end
prePt=[preTrkDat;trackData(1:end-1,:)];
velData(:,2)=sqrt((trackData(:,2)-prePt(:,2)).^2+...
	(trackData(:,3)-prePt(:,3)).^2)./t;    


%to show realtime figure
function ShowRunData(nowPos,handles)
%hfig=getappdata(0,'hfig');
%figure(hfig);
%hold on;
% plot(trackData(:,2),trackData(:,3),'o','MarkerSize',2,...
%     'MarkerEdgeColor','b','MarkerFaceColor','b');
%hold on;
global prePosHandle;
delete(prePosHandle);
prePosHandle=plot(nowPos(:,1),nowPos(:,2),'o','MarkerSize',2,...
    'MarkerEdgeColor','r','MarkerFaceColor','r');


% --- Executes on button press in ctrl_Load.
function ctrl_Load_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
ctrlState=getappdata(0,'ctrlState');
if ctrlState>=3
    handles=init(handles);
    guidata(hObject,handles);
end
ctrlState=getappdata(0,'ctrlState');
if ctrlState==0
    [FileName,pName] = uigetfile({
        '*.mat','Matlab Data File(*.mat)';
        '*.avi','Avi File(*.avi)';
        '*.wmv','Windows media File(*.wmv)';        
        '*.mov','Mac movie File(*.mov)'; 
        '*.*','all file(*.*)';
        },'Load vedio file',PathName);
    if FileName~=0
        PathName=pName;
        fname=fullfile(PathName,FileName);
        setappdata(0,'PathName',PathName);
        setappdata(0,'FileName',FileName);
        set(handles.text_folderName,'String',PathName);
        if strcmpi(FileName(end-3:end),'.mat')
            %read all data
            load(fname,'trackData','scoreData','bkImg2D','ROI','velocityData',...
                'ardInfo','freezingData','moveTag');
            setappdata(0,'trackData',trackData);
            setappdata(0,'scoreData',scoreData);
            setappdata(0,'bkImg2D',bkImg2D);
            setappdata(0,'ROI',ROI);
            setappdata(0,'velocityData',velocityData);
            setappdata(0,'freezingData',freezingData);
            setappdata(0,'moveTag',moveTag);
            if isfield(ardInfo,'totalTime')
                totalTim=ardInfo.totalTime;
            else
                totalTim=velocityData(end,1);
                ardInfo.totalTime=totalTim;
            end
            setappdata(0,'ardInfo',ardInfo);
            setappdata(0,'totalTim',totalTim);
            setappdata(0,'recTime',totalTim);
            updateProtPanel(ardInfo,handles);
            set(handles.edit_RecTime,'String',s2hhmmss(totalTim,0));
                     
            chamberMap=getchambers(ROI,bkImg2D);
            [scoreData,binData,posHotMap]=getScore(chamberMap,trackData);
            %disp(binData);
            %plot background
            %showBackground_ROI(bkImg2D,ROI,handles)
            %plot the result figure 
            freezingTotal=sum(freezingData(:,3));
            set(handles.text_freezingSec,'String',num2str(freezingTotal,'%8.1f'));
            showTrack(trackData,scoreData,bkImg2D,ROI,[],handles);
            plotFreezing(velocityData,freezingData,handles);
            if ~isempty(ROI.x)
                showPosHotMap(posHotMap,scoreData,ROI,handles);
                setappdata(0,'chamberMap',chamberMap);
                setappdata(0,'posHotMap',posHotMap);
                setappdata(0,'binData',binData);
                %also show the curve with bin=5min
                showbinData(binData,handles);
            end
            %check if video is there
            fname2=strcat(fname(1:end-4),'.avi');
            if exist(fname2,'file')>1               %works for higher version
%             if isfile(fname2)               %works for higher version
                aviobj=VideoReader(fname2);
                totalTim=aviobj.Duration;
                setappdata(0,'totalTim',totalTim);
                setappdata(0,'aviobj',aviobj);
                disp('Video found and loaded!');
            end
        else
            aviobj=VideoReader(fname);
            %aviobj=mmreader(fname);
            setappdata(0,'aviobj',aviobj);
            %get some basic information of vedio
            vWid=aviobj.Width;
            vHei=aviobj.Height;
            %chamberMap=zeros(vHei,vWid);        
            totalTim=aviobj.Duration;
            frmRate=round(aviobj.FrameRate);  
            setappdata(0,'totalTim',totalTim);
            nFrames=floor(totalTim*aviobj.FrameRate);
            %get the background image 
            mov=zeros(vHei,vWid,3,1);
            disp('Generating background image from video, please waiting...');
            t1=0;t2=240;fstep=60;         %time windows (sec) to calculate background
            frmnum=floor((t2-t1)*frmRate/fstep);
            for i=1:frmnum
                aviobj.CurrentTime=t1+i*fstep/frmRate;
                mov(:,:,:,i)=readFrame(aviobj);
            end
            %rgb to gray
            mov=mov(:,:,1,:)*0.30+mov(:,:,2,:)*0.59+mov(:,:,3,:)*0.11;
            mov=reshape(mov,[vHei,vWid,size(mov,4)]);
            bkImg=getBackgroundImg(mov);
            setappdata(0,'bkImg2D',bkImg);
            th=getappdata(0,'threshold');          
            %generate threshold
            if th==0
                th=getThreshold(mov,bkImg);
                setappdata(0,'threshold',th);
                set(handles.edit_Threshold,'String',num2str(th));
            end

            %just load the first frame here
            %select the region(ROI),then load all frames
            hfig=getappdata(0,'hfig');
            if isempty(hfig)
                hfig=creatFigure([],vWid,vHei);  
                setappdata(0,'hfig',hfig);
            else
                set(0,'currentfigure',hfig);
            end
            set(hfig,'Name',FileName);
            imshow(bkImg);
            
            nowFrmNum=1;       
            aviobj.CurrentTime=0;
            nowFrame=rgb2gray(readFrame(aviobj));    
            %figure('position',[400,100,vWid,vHei]); 
            setappdata(0,'ctrlState',3);
            frameNums=[nowFrmNum,nFrames,frmRate];            
            set(handles.txt_Timer,'String',s2hhmmss(frameNums(1)/frameNums(3),0));
            setappdata(0,'nowFrame',nowFrame);
            setappdata(0,'frameNums',frameNums);   
        end
        %display some information
        recTime=getappdata(0,'recTime');
        if recTime>totalTim
                recTime=floor(totalTim);
                setappdata(0,'recTime',recTime);
                set(handles.edit_RecTime,'String',s2hhmmss(recTime,0));
        end
    end
end
guidata(hObject,handles);

%read txt-file (data saved)
function [trackData,scoreData,bkgImg,ROI]=readTxtData(fname)
%trackData=[];
%scoreData=[];
%bkgImg=[];
ROI=struct('x',[],'y',[],'width',[],'height',[],'handle',[]);
fid=fopen(fname);
tmpstr=textscan(fid,'%s %s',3);
tmpstr=textscan(fid,'%s %f',1);
dotnum=tmpstr{2}(1);
tmpstr=textscan(fid,'%s %f',1);
chambernum=tmpstr{2}(1);
tmpstr=textscan(fid,'%s',1);
scoreData=fscanf(fid,'%f',[1,chambernum]);
tmpstr=textscan(fid,'%s',1);
ROIDat=fscanf(fid,'%f',[4,chambernum+1])';
bkgImg=uint8(zeros(ROIDat(1,3),ROIDat(1,4),3))+255;
ROI.x=ROIDat(2:end,1);
ROI.y=ROIDat(2:end,2);
ROI.width=ROIDat(2:end,3);
ROI.height=ROIDat(2:end,4);
tmpstr=textscan(fid,'%s',1);
trackData=fscanf(fid,'%f',[3,dotnum])';


%to generate the thresold 
function th=getThreshold(mov,bkImg)
bkTmp=min(mov,[],4);
bkTmp=rgb2gray(bkTmp);
deltaM=bkImg-bkTmp;
th=mean2(deltaM)+std2(deltaM);
%th=0.5*mean2(deltaM)+0.5*max(deltaM(:));

% --- Executes on button press in ctrl_Analyze.
function ctrl_Analyze_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Analyze (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ROI=getappdata(0,'ROI');
bkImg2D=getappdata(0,'bkImg2D');
recTime=getappdata(0,'recTime');
ctrlState=getappdata(0,'ctrlState');
aviobj=getappdata(0,'aviobj');
frameNums=getappdata(0,'frameNums');
detectFreezingTag=getappdata(0,'detectFreezingTag');
frmRate=floor(frameNums(3)+0.1);
frameStep=getappdata(0,'frameStep');
if ctrlState>=3 && ~isempty(aviobj)     
    totalTim=aviobj.Duration;   
    setappdata(0,'totalTim',totalTim);   
    if recTime>0 && recTime<floor(totalTim)
        loadTim=recTime;        
    else
        loadTim=floor(totalTim-1);
    end  
%     if recTime>0 && recTime<floor(totalTim)
%         loadFrames=recTime*frmRate;        
%     else
%         loadFrames=totalTim*frmRate;
%     end    
    trackData=zeros([],3);   
    th=getappdata(0,'threshold');
    aviobj.CurrentTime=0;
    %skip the first 1min
    %aviobj.CurrentTime=30;
    h=waitbar(0,'Generate track from video,please wait...');  
    k=1;
    while hasFrame(aviobj) && k<=loadTim
        %M1=readFrame(aviobj);
        %convert to gray-images
        %M2=M1(:,:,1)*0.30+M1(:,:,2)*0.59+M1(:,:,3)*0.11;
        frms=floor([1,frmRate]+(k-1)*frmRate);
        mov=read(aviobj,frms);
        mov=mov(:,:,:,1:frameStep:end);
        n=floor((frmRate-1)/frameStep)+1;
        ki=floor([1,n]+(k-1)*n);
        trackData(ki(1):ki(2),2:3)=getPosData1(mov,bkImg2D,th); 
        waitbar(k/loadTim,h);
        k=k+1;
    end  
    close(h);  
    trackData(:,1)=(1:length(trackData(:,2)))/frmRate;
    %trackData(:,1)=linspace(1/frmRate,loadTim,length(trackData(:,2)));
    %remove zeros(mouse not found)
    idx=find(trackData(:,2)>0 | trackData(:,3)>0);
    trackData=trackData(idx,:);
    disp('mouse tracking done');
    
    %calculate velocity
    velData=getVelocity(trackData,[]);
    %smooth
    velData(:,2)=smooth(velData(:,2),3);
    setappdata(0,'trackData',trackData);
    setappdata(0,'velocityData',velData);
    setappdata(0,'ctrlState',4);
end
%allow re-analysis without re-do video-processing
trackData=getappdata(0,'trackData');
velData=getappdata(0,'velocityData');
if ~isempty(trackData) && ~isempty(velData)
    if ~isempty(ROI.x)
        %generate chamberMap
        chamberMap=getchambers(ROI,bkImg2D);
        %calculate the index (time spent in chambers)
        [scoreData,binData,posHotMap]=getScore(chamberMap,trackData);
        %show score
        disp(scoreData);
        %figure;
        bar(scoreData);
        %show the hotmap of position (for long-term recording)
        %showPosHotMap(posHotMap,scoreData,ROI,handles);
        %showbinData(binData,handles);
        setappdata(0,'chamberMap',chamberMap);
        setappdata(0,'scoreData',scoreData);
        setappdata(0,'binData',binData);
        setappdata(0,'posHotMap',posHotMap);
    else
        scoreData=[];
    end
    %draw the mouse track
    showTrack(trackData,scoreData,bkImg2D,ROI,[],handles);
    
    freezingPars=getappdata(0,'freezingPars');
    freezingPars(1)=getVelThreshold(velData);
    setappdata(0,'freezingPars',freezingPars);
    set(handles.edit_freezingVth','String',num2str(freezingPars(1),'%6.1f'));
    if detectFreezingTag
        [freezingData,moveTag]=getFreezing(trackData,velData);
        freezingTotal=sum(freezingData(:,end));
        plotFreezing(velData,freezingData,handles);
        set(handles.text_freezingSec,'String',num2str(freezingTotal,'%8.1f'));
        setappdata(0,'freezingTotal',freezingTotal);
        setappdata(0,'freezingData',freezingData);
        setappdata(0,'moveTag',moveTag);
        disp('Freezing episodes(start/end/duration(s))');
        disp(freezingData);
        replay(handles);
    end
end
guidata(hObject,handles);

%replay movie marked with freezing
function replay(handles)
aviobj=getappdata(0,'aviobj');
if isempty(aviobj)
    return;
end
setappdata(0,'adFreezingEpisode',[-1,-1,-1]);        %add or delete a freezing episode [start,end,duration] (sec)
trackData=getappdata(0,'trackData');
velData=getappdata(0,'velocityData');
totalTim=getappdata(0,'totalTim');
frmRate=round(aviobj.FrameRate); 
loadFrames=floor(totalTim*frmRate);
%deal with in-consistent velocityData 
velNum=size(velData,1);
if velNum<loadFrames
    velData(velNum+1:loadFrames,:)=zeros(loadFrames-velNum,2);
    trackData(velNum+1:loadFrames,:)=zeros(loadFrames-velNum,3);
end
% vWid=aviobj.Width;
% vHei=aviobj.Height;
aviobj.CurrentTime=0;
h=getappdata(0,'hreplayer');
if isempty(h)
    h=createReplayFigure(handles);
    setappdata(0,'hreplayer',h);
else
    if ishandle(h)
        %set(0,'CurrentFigure',h);
        figure(h);
    else
        h=createReplayFigure(handles);
        setappdata(0,'hreplayer',h);
    end
end
cls=['r','g'];
jumpStep=10;        %unit=sec
pauseTime=0.02;      %range=0.01-0.1;
isAdjustFreezing=0;
currentFrame=1;
M1=readFrame(aviobj);
while hasFrame(aviobj) && currentFrame<=loadFrames
    figure(h);
    image(M1);
    t=currentFrame/frmRate;
    info=['t = ',num2str(t,'%8.1f'), ' v = ',num2str(velData(currentFrame,2),'%8.1f')];
    text(10,10,info,'color','r');
    plotLocFlag=getappdata(0,'plotLocFlag');
    if plotLocFlag
        rectangle('Position',[trackData(currentFrame,2)-5,trackData(currentFrame,3)-5,10,10],'EdgeColor','r');
    end
    if isfreezingTime(t)>0
        rectangle('Position',[10,40,30,30],'Curvature',[1 1],'FaceColor',[1,1,0],'EdgeColor','y');
    end
    if isAdjustFreezing>0
        rectangle('Position',[10,40,30,30],'Curvature',[1 1],'FaceColor',cls(isAdjustFreezing),'EdgeColor',cls(isAdjustFreezing));
    end
    if mod(currentFrame,10)==1
        sethfreezingPos(t);
    end
    pause(pauseTime);
    replayTag=getappdata(0,'replayTag');
    switch replayTag
        case 0          %no replay - end
            aviobj.CurrentTime=aviobj.Duration;
        case 1          %normal
            currentFrame=currentFrame+1;
            M1=readFrame(aviobj);
        case 2          %pause
            %currentFrame=currentFrame;
        case 3          %jump forward
            aviobj.CurrentTime=min(aviobj.CurrentTime+jumpStep,aviobj.Duration-0.1);
            M1=readFrame(aviobj);
            currentFrame=round(aviobj.CurrentTime*frmRate)+1;
        case 4          %jump backward
            aviobj.CurrentTime=max(0,aviobj.CurrentTime-jumpStep);
            M1=readFrame(aviobj);
            currentFrame=round(aviobj.CurrentTime*frmRate)+1;
        case 5          %slow down replay
            pauseTime=min(0.1,pauseTime+0.01);
        case 6          %speed up
            pauseTime=max(0.01,pauseTime-0.01);
        case 7          %adjust freezing/ADD - start
            isAdjustFreezing=1;
        case 8          %adjust freezing - end
            isAdjustFreezing=0;
        case 9          %adjust freezing/DELETE - start
            isAdjustFreezing=2;
    end
    if replayTag>2
        setappdata(0,'replayTag',1);
    end
    setappdata(0,'currentFrameTime',t);
end


function hfig=createReplayFigure(handles)
hfig=figure();
set(gca,'position',[0,0,1,1]);
set(gcf,'WindowKeyPressFcn',{@figReplayKeyPress,handles});


function sethfreezingPos(t)
htPos=getappdata(0,'hcurrentPos');
hfreezing=getappdata(0,'hfreezing');
set(0,'CurrentFigure',hfreezing);
ylim=get(gca,'ylim');
if isempty(htPos)
    htPos=line([1,1]*t,[ylim(1),ylim(2)],'color','r');
else
    delete(htPos);
    htPos=line([1,1]*t,[ylim(1),ylim(2)],'color','r');
end
setappdata(0,'hcurrentPos',htPos);

%Hot-keys: to do when press keys in the image-figure
function figReplayKeyPress(hObject, eventdata, handles)
switch hObject.CurrentCharacter
    case 'l'                %to mark animal location
        plotLocFlag=getappdata(0,'plotLocFlag');
        setappdata(0,'plotLocFlag',~plotLocFlag);
    case '.'                %key='>', jump forward
        setappdata(0,'replayTag',3);
    case ','                %key='<', jump background
        setappdata(0,'replayTag',4);
    case '-'                %key='-', slow down
        setappdata(0,'replayTag',5);
    case '='                %key='+', speed up
        setappdata(0,'replayTag',6);
    case 'p'                %pause at the current frame
        replayTag=getappdata(0,'replayTag');
        if replayTag==2
            setappdata(0,'replayTag',1);
        else
            setappdata(0,'replayTag',2);
        end
    case 'a'            %add a freezing episode
        adjustFreezing(1,handles);
    case 'd'            %delete a freezing episode
        adjustFreezing(2,handles);
    case 'c'            %cancel editing
        adjustFreezing(0,handles);
end

function adjustFreezing(action,handles)
adFreezingEpisode=getappdata(0,'adFreezingEpisode');
currentFrameTime=getappdata(0,'currentFrameTime');
if adFreezingEpisode(1)>0
	adFreezingEpisode(2)=currentFrameTime;
    adFreezingEpisode(3)=adFreezingEpisode(2)-adFreezingEpisode(1);
    setappdata(0,'replayTag',8);
    %update freezingData
    updatefreezingData(action,adFreezingEpisode,handles);
    %init for next action
    adFreezingEpisode=[0,-1,-1];
else
	adFreezingEpisode(1)=currentFrameTime;
    if action==1
        setappdata(0,'replayTag',7);
    elseif action==2
        setappdata(0,'replayTag',9);
    end
end
if action==0
   adFreezingEpisode=[0,-1,-1];
   setappdata(0,'replayTag',8);
end
setappdata(0,'adFreezingEpisode',adFreezingEpisode);


function updatefreezingData(action,adFreezingEpisode,handles)
freezingData=getappdata(0,'freezingData');
idx1=isfreezingTime(adFreezingEpisode(1));
idx2=isfreezingTime(adFreezingEpisode(2));
%fnum=size(freezingData,1);
if action==1            %add
    %freezingData(end+1,:)=adFreezingEpisode;
    if idx1>0 && idx2>0
        if idx1==idx2
            %do nothing, already marked as freezing 
        else
            freezingData(idx1,2)=freezingData(idx2,2);
            freezingData(idx1,3)=freezingData(idx1,2)-freezingData(idx1,1);
            freezingData(idx2,2)=freezingData(idx2,1);
            freezingData(idx2,3)=0;
        end
    elseif idx1>0 && idx2==0
        freezingData(idx1,2)=adFreezingEpisode(2);
        freezingData(idx1,3)=freezingData(idx1,2)-freezingData(idx1,1);
    elseif idx1==0 && idx2>0
        freezingData(idx2,1)=adFreezingEpisode(1);
        freezingData(idx2,3)=freezingData(idx2,2)-freezingData(idx2,1);
    else
        freezingData(end+1,:)=adFreezingEpisode;
    end
    idx0=freezingData(:,1)>adFreezingEpisode(1) & freezingData(:,2)<adFreezingEpisode(2);
	if ~isempty(find(idx0))
        freezingData=freezingData(~idx0,:);
	end
elseif action==2        %delete
    if idx1>0 && idx2>0
        if idx1==idx2
            fep=freezingData(idx1,:);
            freezingData(idx1,2)= adFreezingEpisode(1);
            freezingData(idx1,3)=freezingData(idx1,2)-freezingData(idx1,1);
            freezingData(end+1,1)=adFreezingEpisode(2);
            freezingData(end,2)=fep(2);
            freezingData(end,3)=freezingData(end,2)-freezingData(end,1);
        else
            freezingData(idx1,2)=adFreezingEpisode(1);
            freezingData(idx1,3)=freezingData(idx1,2)-freezingData(idx1,1);
            freezingData(idx2,1)=adFreezingEpisode(2);
            freezingData(idx1,3)=freezingData(idx2,2)-freezingData(idx2,1);
        end
    elseif idx1>0 && idx2==0
            freezingData(idx1,2)=adFreezingEpisode(1);
            freezingData(idx1,3)=freezingData(idx1,2)-freezingData(idx1,1);
    elseif idx1==0 && idx2>0
            freezingData(idx2,1)=adFreezingEpisode(2);
            freezingData(idx2,3)=freezingData(idx2,2)-freezingData(idx2,1);
    else
        %see below
    end
    idx0=freezingData(:,1)>adFreezingEpisode(1) & freezingData(:,2)<adFreezingEpisode(2);
	if ~isempty(find(idx0))
        freezingData=freezingData(~idx0,:);
    end
end
setappdata(0,'freezingData',freezingData);
freezingTotal=sum(freezingData(:,3));
setappdata(0,'freezingTotal',freezingTotal);
set(handles.text_freezingSec,'String',num2str(freezingTotal,'%8.1f'));
%update plot
velData=getappdata(0,'velocityData');
plotFreezing(velData,freezingData,handles);


function flag=isfreezingTime(t)
freezingData=getappdata(0,'freezingData');
flag=0;
i1=freezingData(:,1)>t;
i2=freezingData(:,2)>t;
idx=find(xor(i1,i2));
if ~isempty(idx)
    flag=idx(1);
end
%slower method
% n=size(freezingData,1);
% for i=1:n
%     if t>=freezingData(i,1) && t<=freezingData(i,2)
%         flag=1;
%         break;
%     end
% end

%get mouse position by subtraction
function posData=getPosData0(mov,bkImg,th)
%mov=gray-images, bkImg=gray-image [H,W,frm]
[vH,vW,chs,frmnum]=size(mov);
%frames=mov(:,:,1,:)*0.30+mov(:,:,2,:)*0.59+mov(:,:,3,:)*0.11;
frames=reshape(mov,vH,vW,frmnum);
posData=zeros(frmnum,2);
bk=repmat(bkImg,[1,1,frmnum]);
M=(bk-frames)>th;
%M=(frames-bk)>th;              %for white mice    
M=bwareaopen(M,20);        %remove sparse noise
%set the center as the position 
for i=1:frmnum
    M2=bwareaopen(M(:,:,i),100);        %remove sparse noise
    SS=bwconncomp(M2,4);
    %get the biggest area as the mouse
    if SS.NumObjects>0
        cc=regionprops(SS,'Centroid','Area');
        [mx,mxIdx]=max(extractfield(cc,'Area'));
        posData(i,1:2)=cc(mxIdx).Centroid';
    end
end
clear bk frames M;
clear t x y yt;

%get mouse position by subtraction
function posData=getPosData1old(mov,bkImg,th)
%mov=gray-images, bkImg=gray-image [H,W,frm]
[vH,vW,chs,frmnum]=size(mov);
%frames=mov(:,:,1,:)*0.30+mov(:,:,2,:)*0.59+mov(:,:,3,:)*0.11;
frames=reshape(mov,vH,vW,frmnum);
posData=zeros(frmnum,2);
bk=repmat(bkImg,[1,1,frmnum]);
M=(bk-frames)>th;
%M=(frames-bk)>th;              %for white mice    
M=bwareaopen(M,50);        %remove sparse noise
%try to connect open-circles
se=strel('disk',2);
M=imdilate(M,se);
%set the center as the position 
for i=1:frmnum
    %M2=bwareaopen(M(:,:,i),100);        %remove sparse noise
    SS=bwconncomp(M(:,:,i),8);
    %get the biggest area as the mouse
    if SS.NumObjects>0
        cc=regionprops(SS,'Centroid','Area');
        [mx,mxIdx]=max(extractfield(cc,'Area'));
        posData(i,1:2)=cc(mxIdx).Centroid';
    end
end
clear bk frames M;
clear t x y yt;

%get mouse position by subtraction (one mouse)-actMonitor3.0-version
function posData=getPosData1(mov,bkImg,th)
%mov=rgb, bkImg=gray-image [H,W,frm]
[vH,vW,chs,frmnum]=size(mov);       %chs=1 for gray-image
if chs>1
    frames=double(mov(:,:,1,:)*0.30+mov(:,:,2,:)*0.59+mov(:,:,3,:)*0.11); %rgb-video
else
    frames=double(mov);
end
frames=reshape(frames,vH,vW,frmnum);
posData=zeros(frmnum,2);
bk=double(repmat(bkImg,[1,1,frmnum]));
M=(bk-frames)>th;           %for both black 
M=bwareaopen(M,100);        %remove sparse noise
%select the largest object as the mouse
for k=1:frmnum
	SS=bwconncomp(M(:,:,k),8);
	if SS.NumObjects>0
        cc=regionprops(SS,'Centroid','Area');
        ar=cat(1,cc.Area);
    	[a2,arIdx]=sort(ar,'descend');
    	posData(k,1:2)=cc(arIdx(1)).Centroid;
    end
end
clear bk frames M;
clear t x y yt;


function [freezingData,moveTag]=getFreezing0(trackData,velData)
c0th=0.1;        %threshold for sleep/freezing judge, default[0.05,0.2] for non-cable; use bigger for cable (noisy) 
binT=1.0;        %bin-time window to calculate freezing time
stepT=0.3;      %step/resolution to calculate freezing time

totalSec=velData(end,1);
moveTag=zeros(floor(totalSec/stepT),4);     %[t,moveState,mean-v,v2]
num=1;
for t=0:stepT:totalSec
    idx1=find(velData(:,1)>=t-stepT/2 & velData(:,1)<t+stepT/2);
    mv=mean(velData(idx1,2));
    idx2=find(trackData(:,1)>=t-stepT/2 & trackData(:,1)<t+stepT/2);
    vd=sqrt((trackData(idx2(end),2)-trackData(idx2(1),2)).^2+...
            (trackData(idx2(end),3)-trackData(idx2(1),3)).^2)./stepT;
    %important for sleep/freezing-detection!
    if mv>15 && vd>mv*0.2
        moveTag(num,2)=1;
    else
        moveTag(num,2)=0;
    end
    moveTag(num,1)=t; 
    moveTag(num,3)=mv;
    moveTag(num,4)=vd;
    num=num+1;
end
freezingTotal=0;
freezingData=[0,0,0];
frznum=1;
for t=0:stepT:totalSec
    idx3=find(moveTag(:,1)>=t-binT/2 & moveTag(:,1)<t+binT/2);
    c0=mean(moveTag(idx3,2));
    if c0<c0th
        freezingTotal=freezingTotal+stepT;
        if freezingData(frznum,1)==0
            freezingData(frznum,1)=t;
        end
    else
        if freezingData(frznum,1)>0
            freezingData(frznum,2)=t;
            freezingData(frznum,3)=freezingData(frznum,2)-freezingData(frznum,1);
            frznum=frznum+1;
            freezingData(frznum,:)=zeros(1,3);
        end
    end
end
%connect two episodes if gap less than binT
%freezingData=mergeEpisodes(freezingData,binT);
%remove very short episodes
freezingPars=getappdata(0,'freezingPars');
idx=find(freezingData(:,3)>=freezingPars(2));
freezingData=freezingData(idx,:);
%disp(freezingData);
%disp([freezingTotal,sum(freezingData(:,3))]);



function [freezingData,moveTag]=getFreezing(trackData,velData)
binT=1.0;        %bin-time window to calculate freezing time
frmT=mean(trackData(2:end,1)-trackData(1:end-1,1));
freezingPars=getappdata(0,'freezingPars');

for i=1:size(velData,1)
    t=velData(i,1);
    idx2=find(trackData(:,1)>=t-binT/2 & trackData(:,1)<t+binT/2);
    velData(i,3)=sqrt((trackData(idx2(end),2)-trackData(idx2(1),2)).^2+...
            (trackData(idx2(end),3)-trackData(idx2(1),3)).^2)./binT;
end

moveState=velData(:,2)>freezingPars(1) & velData(:,3)>1;
%remove small gap (<0.1s)
gapnum=round(0.5/frmT);     
moveState=smallsegRemove(moveState,gapnum,0);
moveTag=velData(:,1);
moveTag(:,2)=moveState;
blocks=getBlocks(moveState,0);
freezingData=zeros(size(blocks,1),3);
freezingData(:,1)=velData(blocks(:,2),1);
freezingData(:,2)=velData(blocks(:,3),1);
freezingData(:,3)=freezingData(:,2)-freezingData(:,1);

%remove very short episodes
idx=find(freezingData(:,3)>=freezingPars(2));
freezingData=freezingData(idx,:);
%disp(freezingData);
   

%calculate v-th
function vth=getVelThreshold(velData)
binT=1.0;        %bin-time window to calculate freezing time
frmT=mean(velData(2:end,1)-velData(1:end-1,1));
binnum=round(binT/frmT);
v2=smooth(velData(:,2),binnum);
vr=velData(:,2)./v2;
th1=nanmean(vr)+nanstd(vr);
idx=find(vr>th1);
vth=prctile(velData(idx,2),10);

%get chamberMap from ROI
function chamberMap=getchambers(ROI,bkImg)
[vH,vW]=size(bkImg);
chamberMap=zeros(vH,vW);
num=length(ROI.x);
for i=1:num
    x=[ROI.x(i),ROI.x(i)+ROI.width(i),ROI.x(i)+ROI.width(i),ROI.x(i),ROI.x(i)];
    y=[ROI.y(i),ROI.y(i),ROI.y(i)+ROI.height(i),ROI.y(i)+ROI.height(i),ROI.y(i)];
    bk1=poly2mask(x,y,vH,vW);
    chamberMap=chamberMap+bk1*i;
end

%calculate the score (time spent in each chamber)
function [scoreData,binData,posHotMap]=getScore(chamberMap,trackData)
if isempty(chamberMap)
    scoreData=[];
    binData=[];
    posHotMap=[];
else
    chamberNum=max(chamberMap(:));
    scoreData=zeros(1,chamberNum);
    posHotMap=0*chamberMap;
    binTim=5*60;            %time bin for curve
    dn=ceil(trackData(end,1)/binTim);
    binData=zeros(dn,chamberNum);
    ptnum=length(trackData);
    for i=1:ptnum
        k=ceil(trackData(i,1)/binTim);
        x=floor(trackData(i,2));
        y=floor(trackData(i,3));
        if x>0 && y>0
            posHotMap(y,x)=posHotMap(y,x)+1;
            px=chamberMap(y,x);
            if px>0
                scoreData(px)=scoreData(px)+1;
                binData(k,px)=binData(k,px)+1;
            end
        end
    end
    scoreData=scoreData./ptnum;
    for i=1:dn
        binData(i,:)=binData(i,:)*dn/ptnum;
    end
%     if chamberNum==1
%         scoreData=scoreData./ptnum;
%     else
%         scoreData=scoreData./sum(scoreData);
%     end
%     for i=1:dn
%         if chamberNum==1
%             binData(i,:)=binData(i,:)*dn/ptnum;
%         else
%             binData(i,:)=binData(i,:)/sum(binData(i,:));
%         end
%     end
    posHotMap=posHotMap./(max(posHotMap(:)));
    %smooth the hotmap
    h=fspecial('gaussian',7,1);
    posHotMap=imfilter(posHotMap,h);
    %figure;imagesc(posHotMap);
end

%to draw the track
function showTrack(trackData,scoreData,backgroundImg,ROI,FileName,handles)
hfig=getappdata(0,'hfig');
if isempty(hfig)
	[vHei,vWid]=size(backgroundImg);
	hfig=creatFigure([],vWid,vHei);  
	setappdata(0,'hfig',hfig);
end
figure(hfig);
%show the scoreData
if ~isempty(ROI.x)
    %whitebk=backgroundImg*0+255;
    showBackground_ROI(backgroundImg,ROI);
    n=length(ROI.x);
    w0=min(ROI.width);
    c1=[0.5,0,0];
    c2=[0.75,0.75,0.75];
    for i=1:n
        %horizontal
%         rectangle('position',[ROI.x(i),ROI.y(i)-12,w0*scoreData(i)+0.01,10],...
%             'EdgeColor',c1,'FaceColor', c1);
        text(ROI.x(i),ROI.y(i)-6,num2str(round(scoreData(i)*100)/100),'color',c2);        
        %or vertical
%          rectangle('position',[ROI.x(i)+ROI.width(i),ROI.y(i),10,w0*scoreData(i)],...
%             'EdgeColor',c1,'FaceColor', c1);       
        %text(ROI.x(i)+ROI.width(i),ROI.y(i),num2str(round(scoreData(i)*100)/100),'color',c2);
    end
else
    imshow(backgroundImg);
end
hold all;
%plot(trackData(2:end,2),trackData(2:end,3),'-k','LineWidth',1);
plot(trackData(:,2),trackData(:,3),'-o','MarkerSize',1,'MarkerEdgeColor','b','color',[0.5,0.5,0.5]);
%save the image be default
if ~isempty(FileName)
    fn=strcat(FileName(1:end-4),'.jpg');
    F=getframe(gcf);
    imwrite(F.cdata,fn);
end

%draw the hotmap of position
function showPosHotMap(posHotMap,scoreData,ROI,handles)
clim=[0,mean2(posHotMap)+2*std2(posHotMap)];
[h,w]=size(posHotMap);
figure('position',[200,100,w,h]);
set(gca,'position',[0,0,1,1]);
%ROI
n=length(ROI.x);
if n>0
    %hotmap for ROIs
%     for i=1:n
%         hotROI=posHotMap(ROI.y(i):ROI.y(i)+ROI.height(i),ROI.x(i):ROI.x(i)+ROI.width(i));        
%         imagesc(ROI.x(i),ROI.y(i),hotROI,clim);
%         hold on;
%     end
    imagesc(posHotMap,clim);
    axis image off;
    axis([0,w,0,h]);
    colormap('jet');
    set(gca,'clim',[0,0.02])
    c1=[0.5,0.5,0.5];
    c2=[0.75,0.75,0.75];
    c1='w';c2='w';
    w0=min(ROI.width);
    %cls=get(gca,'colororder');
	for i=1:n
        rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
               'LineStyle','--','edgecolor',[1,1,1]); 
%         rectangle('position',[ROI.x(i),ROI.y(i)-12,w0*scoreData(i)+0.01,10],...
%             'EdgeColor',c1,'FaceColor', c1);
        text(ROI.x(i),ROI.y(i)-6,num2str(round(scoreData(i)*100)/100),'color',c2);             
    end
else
    imagesc(posHotMap,clim);
    colormap('jet');
    set(gca,'clim',[0,0.05])
end

function showbinData(binData,handles)
figure;
%plot(binData);
bar(binData,'stacked');
xlabel('Time(bin#)');
ylabel('Time (%) in each chamber per 5min');

function plotFreezing(velData,freezingData,handles)
hfreezing=getappdata(0,'hfreezing');
if isempty(hfreezing)
    hfreezing=figure('position',[50,50,1000,300]);  
	setappdata(0,'hfreezing',hfreezing);
end
h0=prctile(velData(:,2),90)*3;
c0=[0.75,0.75,1];
figure(hfreezing);cla;
%mark freezing episodes
for i=1:size(freezingData,1)
    rectangle('Position',[freezingData(i,1),0,freezingData(i,3),h0],'FaceColor',c0,'EdgeColor',c0)
end
%plot velocity

    hold on;
    plot(velData(:,1),smooth(velData(:,2),15),'k');
    set(gca,'ylim',[0,h0],'xlim',[0,velData(end,1)]);

xlabel('Time (sec)');
ylabel('Moving speed (pixel/sec)');


% --- Executes on button press in ctrl_Pathname.
function ctrl_Pathname_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Pathname (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
PathName = uigetdir(PathName);
if ~strcmpi(PathName(length(PathName)),'\')
    PathName=strcat(PathName,'\');
end
setappdata(0,'PathName',PathName);
set(handles.text_folderName,'String',PathName);


%converter the time-format
function timstr=s2hhmmss(tim,sTag)
ss=mod(tim,60);
if ~sTag
    ss=floor(ss);
end
mm=mod(floor(tim/60),60);
hh=floor(tim/60/60);
if ss<10
    s=['0',num2str(ss)];
else
    s=num2str(ss);
end
if mm<10
    m=['0',num2str(mm)];
else
    m=num2str(mm);
end
if hh<10
    h=['0',num2str(hh)];
else
    h=num2str(hh);
end
timstr=[h,':',m,':',s];

%converter the time-format
function tim=hhmmss2s(timstr)
if length(timstr)==8
    tim=str2double(timstr(1:2))*60*60+str2double(timstr(4:5))*60+str2double(timstr(7:8));
else
    tim=0;
end

%get filename by current-time
function FileName=getFileName(extName)
nowstr=datestr(now,31);
nowstr(strfind(nowstr,':'))='-';
FileName=strcat(nowstr,extName);

function edit_RecTime_Callback(hObject, eventdata, handles)
% hObject    handle to edit_RecTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_RecTime as text
%        str2double(get(hObject,'String')) returns contents of edit_RecTime as a double
timS=get(hObject,'String');
recTime=hhmmss2s(timS);
setappdata(0,'recTime',recTime);

% --- Executes during object creation, after setting all properties.
function edit_RecTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_RecTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ctrl_Snapshot.
function ctrl_Snapshot_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Snapshot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ctrlState=getappdata(0,'ctrlState');
if ctrlState<=1  %only when none/preview states
     vidobj=getappdata(0,'vidobj');
     bkImgBuffer=getappdata(0,'bkImgBuffer');
     vRes = get(vidobj, 'VideoResolution'); 
%      src = getselectedsource(vidobj); 
%      srcInfo=get(src);
%      if isfield(srcInfo,'Exposure')
%          src.Exposure=-5;
%      end
%      if isfield(srcInfo,'Brightness')
%          src.Brightness=160;
%      end
     snapshot = getsnapshot(vidobj);  
     hfig=getappdata(0,'hfig');
     hfig=creatFigure(hfig,vRes(1),vRes(2));
     image(snapshot);
     colormap(gray(256));
     nowFrame=snapshot;
     [vH,vW,a]=size(snapshot);
     chamberMap=zeros(vH,vW);
     if isempty(bkImgBuffer)
         bfn=0;
     else
        bfn=size(bkImgBuffer,3);
     end
     bkImgBuffer(:,:,bfn+1)=uint8(snapshot);
     bkImg=getBackgroundImg(bkImgBuffer(:,:,:));
     setappdata(0,'chamberMap',chamberMap);
     setappdata(0,'nowFrame',nowFrame);
     setappdata(0,'bkImg2D',bkImg);
     setappdata(0,'hfig',hfig);
elseif ctrlState==3             %when video loaded, click to generate background 
    totalTim=getappdata(0,'totalTim');
    bkImg2D=getappdata(0,'bkImg2D');
    aviobj=getappdata(0,'aviobj');
    [vHei,vWid]=size(bkImg2D);
    frmRate=round(aviobj.FrameRate);  
    %generate a random start timepoints
    t1=rand(1)*totalTim*0.9;
    t2=t1+30;
    fstep=10;  
	frmnum=floor((t2-t1)*frmRate/fstep);
    mov=zeros(vHei,vWid,3,1);
	for i=1:frmnum
        aviobj.CurrentTime=t1+i*fstep/frmRate;
        mov(:,:,:,i)=readFrame(aviobj);
    end
	%rgb to gray
	mov=mov(:,:,1,:)*0.30+mov(:,:,2,:)*0.59+mov(:,:,3,:)*0.11;
	mov=reshape(mov,[vHei,vWid,size(mov,4)]);
	bkImg=getBackgroundImg(mov);
	setappdata(0,'bkImg2D',bkImg);
    hfig=getappdata(0,'hfig');
    if isempty(hfig)
        hfig=creatFigure([],vWid,vHei);  
        setappdata(0,'hfig',hfig);
    else
    set(0,'currentfigure',hfig);
    end
    imshow(bkImg);
end


% --- Executes on button press in ctrl_Clear.
function ctrl_Clear_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%clear ROI-panel
set(handles.listbox_ROI,'String',[]);
set(handles.text_ROI_areaA,'BackgroundColor',[0.9,0.9,0.9]);
set(handles.text_ROI_areaB,'BackgroundColor',[0.9,0.9,0.9]);
set(handles.text_freezingSec,'String','0');
hfig=getappdata(0,'hfig');
if ishandle(hfig)
    delete(hfig);
end
hreplayer=getappdata(0,'hreplayer');
if ishandle(hreplayer)
    delete(hreplayer);
end
hfreezing=getappdata(0,'hfreezing');
if ishandle(hfreezing)
    delete(hfreezing);
end
init(handles);


function edit_Threshold_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Threshold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Threshold as text
%        str2double(get(hObject,'String')) returns contents of edit_Threshold
%        as a double
threshold=str2double(get(hObject,'String'));
setappdata(0,'threshold',threshold);
mROIData=getappdata(0,'mROIData');
ctrlState=getappdata(0,'ctrlState');
if ctrlState==4 && ~isempty(mROIData)
    getDataShow(mROIData,handles);
end
ctrlRecordTag=getappdata(0,'ctrlRecordTag');
bkImg=getappdata(0,'bkImg2D');
vidobj=getappdata(0,'vidobj');
if ~ctrlRecordTag && ~isempty(bkImg) 
    snapshot=[];
    if ~isempty(vidobj)
        snapshot = getsnapshot(vidobj);  
    end
    if ctrlState==3
        snapshot = getappdata(0,'nowFrame');
        %snapshot = rgb2gray(snapshot);  %already did before
    end
    if ~isempty(snapshot)
        M=(bkImg-snapshot)>threshold;           %for black mice
        hfig=getappdata(0,'hfig');
        set(0,'currentfigure',hfig);
        imshow(M);axis off;
    end
end
 

% --- Executes during object creation, after setting all properties.
function edit_Threshold_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Threshold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_Camera_Zoom_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of
%        slider
handles.camZoom=get(hObject,'Value');
set(handles.edit_Camera_Zoom,'String',num2str(handles.camZoom));
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Zoom = handles.camZoom;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_Camera_Zoom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_Camera_Zoom_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Camera_Zoom as text
%        str2double(get(hObject,'String')) returns contents of
%        edit_Camera_Zoom as a double
handles.camZoom=str2double(get(hObject,'String'));
set(handles.slider_Camera_Zoom,'value',handles.camZoom);
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Zoom = handles.camZoom;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_Camera_Zoom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_Camera_Pan_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.camPan=get(hObject,'Value');
set(handles.edit_Camera_Pan,'String',num2str(handles.camPan));
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Pan = handles.camPan;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_Camera_Pan_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_Camera_Pan_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Camera_Pan as text
%        str2double(get(hObject,'String')) returns contents of edit_Camera_Pan as a double
handles.camPan=str2double(get(hObject,'String'));
set(handles.slider_Camera_Pan,'value',handles.camPan);
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Pan = handles.camPan;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_Camera_Pan_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_Camera_Tilt_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.camTilt=get(hObject,'Value');
set(handles.edit_Camera_Tilt,'String',num2str(handles.camTilt));
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Tilt = handles.camTilt;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_Camera_Tilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_Camera_Tilt_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Camera_Tilt as text
%        str2double(get(hObject,'String')) returns contents of edit_Camera_Tilt as a double
handles.camTilt=str2double(get(hObject,'String'));
set(handles.slider_Camera_Tilt,'value',handles.camTilt);
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Tilt = handles.camTilt;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_Camera_Tilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in listbox_vFormats.
function listbox_vFormats_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_vFormats (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_vFormats contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_vFormats
vFormats=cellstr(get(hObject,'String'));
vnum=get(hObject,'Value');
vidobj=getappdata(0,'vidobj');
if ~isempty(vidobj)
    delete(vidobj);
end
camDevices=getappdata(0,'camDevices');
camID=get(handles.listbox_camList,'Value');
vidobj = createVidObj(camDevices(camID),vFormats{vnum},handles);
setappdata(0,'vidobj',vidobj);
%guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function listbox_vFormats_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_vFormats (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%clear all-data from memory
rmappdata(0,'trackData');
rmappdata(0,'bkImg2D');
rmappdata(0,'chamberMap');
rmappdata(0,'scoreData');

%clear the camera
vidobj=getappdata(0,'vidobj');
delete(vidobj);
rmappdata(0,'vidobj');

%Arduino
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    fclose(myArduino);
    delete(myArduino);
    rmappdata(0,'myArduino');
end



% --- Executes on button press in checkbox_VideosaveTag.
function checkbox_VideosaveTag_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_VideosaveTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_VideosaveTag
VideoSaveTag=get(hObject,'Value');
setappdata(0,'VideoSaveTag',VideoSaveTag);

% --- Executes on button press in checkbox_OnlineTag.
function checkbox_OnlineTag_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_OnlineTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkbox_OnlineTag
OnlineTag=get(hObject,'Value');
setappdata(0,'OnlineTag',OnlineTag);


% --- Executes on selection change in listbox_Arduino_portNames.
function listbox_Arduino_portNames_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_Arduino_portNames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns listbox_Arduino_portNames contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_Arduino_portNames
list=cellstr(get(hObject,'String'));
num=get(hObject,'Value');
setappdata(0,'serialPort',list{num});


% --- Executes during object creation, after setting all properties.
function listbox_Arduino_portNames_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_Arduino_portNames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_Arduino_Connect.
function pushbutton_Arduino_Connect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Arduino_Connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if strcmpi(hObject.String,'Connect')
    serialPort=getappdata(0,'serialPort');
    myArduino=serial(serialPort,'BaudRate',9600);
    %open the port
    fopen(myArduino);
    pause(2); 
    disp(['Connected to Arduino on ',serialPort]);

    %update arduino
    ardInfo=getappdata(0,'ardInfo');
    updateArduino(myArduino, ardInfo);
    setappdata(0,'myArduino',myArduino);
    set(hObject,'String','Disconnect');
elseif strcmpi(hObject.String,'Disconnect')
    myArduino=getappdata(0,'myArduino');
    if ~isempty(myArduino)
        fclose(myArduino);
        delete(myArduino);
        setappdata(0,'myArduino',[]);
        disp('Disconnected from Arduino');
    end
    set(hObject,'String','Connect');
end

function updateArduino(myArduino, ardInfo)
%update arduino
if ardInfo.toneFlag
	cmdStr=['t',num2str(ardInfo.toneDuration),'/'];
    fwrite(myArduino,cmdStr,'uchar');
end
if ardInfo.shockFlag
	cmdStr=['s',num2str(ardInfo.shockDuration),'/'];
	fwrite(myArduino,cmdStr,'uchar');
end

% --- Executes on button press in pushbutton_Arduino_Test.
function pushbutton_Arduino_Test_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Arduino_Test (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    ardInfo=getappdata(0,'ardInfo');
    % send commands to arduino
    if ardInfo.toneFlag
        fwrite(myArduino,'T','uchar'); 
    end
    if ardInfo.shockFlag
        fwrite(myArduino,'S','uchar'); 
    end
    if ardInfo.laserFlag
        fwrite(myArduino,'X','uchar');      
    end
end

% --- Executes on button press in pushbutton_Arduino_Stop.
function pushbutton_Arduino_Stop_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Arduino_Stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    fwrite(myArduino,'C','uchar');      %cancel/stop tone and shock if delivering 
end

% --- Executes on button press in checkbox_replay.
function checkbox_replay_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_replay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkbox_replay
val=get(hObject,'Value');
setappdata(0,'replayTag',val);
if val 
    replay(handles);
end


% --- Executes on button press in pushbutton_ROI_Add.
function pushbutton_ROI_Add_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_Add (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
taskTag=getappdata(0,'taskTag');
nowFrame=getappdata(0,'nowFrame');
ctrlState=getappdata(0,'ctrlState');
bkImg=getappdata(0,'bkImg2D');
ROI=getappdata(0,'ROI');
ADDROIflag=0;
if ctrlState>=3 || ~isempty(nowFrame)  
    hfig=getappdata(0,'hfig');
    if ishandle(hfig)
        figure(hfig);
    end
    ADDROIflag=1;
elseif ctrlState==0 || ~isempty(bkImg)  
    showBackground_ROI(bkImg,ROI,handles);
    ADDROIflag=1;
end
if ADDROIflag
    cls=get(gca,'colororder');
    n=length(ROI.x)+1;
    %set(handles.figure1,'pointer','crosshair');
    [x,y] = ginput(2);
    x1=min(x);
    y1=min(y);
    w=abs(x(2)-x(1));
    h=abs(y(2)-y(1));
    %for open field only, re-adjust to the center (50%)
    if taskTag==1
    %     x1=x1+w/4;y1=y1+h/4;
    %     w=w/2;h=h/2;
        %for NOR task
        nor_wid=460/3;     %width of object-related area
        %nor_wid=1080/3;     %width of object-related area
        x1=mean(x)-nor_wid/2;
        y1=mean(y)-nor_wid/2;
        w=nor_wid;h=w;
    end
    handle=rectangle('Position',[x1,y1,w,h],'LineStyle','--','edgecolor',cls(n+1,:));
    ROI=struct('x',[ROI.x x1],'y',[ROI.y y1],'width',[ROI.width w],'height',[ROI.height h],...
        'handle',[ROI.handle handle]);   
    setappdata(0,'ROI',ROI);
    %add the listbox
    listname=cell(1,n);
    for i=1:n
        listname{i}=strcat('Area',num2str(i));
    end
    set(handles.listbox_ROI,'String',listname);
    set(handles.listbox_ROI,'Value',n);
    %update the color label    
    if n==1
        set(handles.text_ROI_areaA,'BackgroundColor',cls(n+1,:));
    elseif n==2
        set(handles.text_ROI_areaB,'BackgroundColor',cls(n+1,:));
    end
    %get chamberMap for online-analysis
    
    chamberMap=getchambers(ROI,bkImg);
    setappdata(0,'chamberMap',chamberMap);
    setappdata(0,'ctrlROIShowTag',1);
    set(handles.pushbutton_ROI_ShowHide,'String','Hide');
end


% --- Executes on button press in pushbutton_ROI_Remove.
function pushbutton_ROI_Remove_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_Remove (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%remove from listbox
listname=cellstr(get(handles.listbox_ROI,'String'));
val=get(handles.listbox_ROI,'Value');
listnum=length(listname);
idx=find((1:listnum)~=val);
if isempty(idx)
    set(handles.listbox_ROI,'String',[])
else
    listname=listname(idx);
    set(handles.listbox_ROI,'String',listname)
    set(handles.listbox_ROI,'Value',listnum-1)
end

%remove from ROI
ROI=getappdata(0,'ROI');
if ~isempty(ROI.handle)
    delete(ROI.handle(val));
end
ROI=struct('x',ROI.x(idx),'y',ROI.y(idx),'width',ROI.width(idx),...
    'height',ROI.height(idx),'handle',ROI.handle(idx));
setappdata(0,'ROI',ROI);

%remov the color label
if val==1
	set(handles.text_ROI_areaA,'BackgroundColor',[0.9,0.9,0.9]);
elseif val==2
	set(handles.text_ROI_areaB,'BackgroundColor',[0.9,0.9,0.9]);
end

% --- Executes on selection change in listbox_ROI.
function listbox_ROI_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_ROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_ROI contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_ROI


% --- Executes during object creation, after setting all properties.
function listbox_ROI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_ROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_ROI_ShowHide.
function pushbutton_ROI_ShowHide_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_ShowHide (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ctrlROIShowTag=getappdata(0,'ctrlROIShowTag');
ctrlROIShowTag=mod(ctrlROIShowTag+1,2);
setappdata(0,'ctrlROIShowTag',ctrlROIShowTag);
ROI=getappdata(0,'ROI');
rn=length(ROI.x);
hfig=getappdata(0,'hfig');
if ishandle(hfig)
    figure(hfig);
end
if ctrlROIShowTag
    cls=get(gca,'colororder');
    set(handles.pushbutton_ROI_ShowHide,'String','Hide');
    for i=1:rn
        h=rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
            'LineStyle','--','edgecolor',cls(i+1,:));
        ROI.handle(i)=h;
    end
else
    set(handles.pushbutton_ROI_ShowHide,'String','Show');
    for i=rn:-1:1
        delete(ROI.handle(i));
        ROI.handle(i)=[];
    end
end
setappdata(0,'ROI',ROI);


% --- Executes on button press in pushbutton_ROI_Histgram.
function pushbutton_ROI_Histgram_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_Histgram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ROI=getappdata(0,'ROI');
I=getappdata(0,'nowFrame');
n=length(ROI.x);
if n>0
    bkImg=rgb2gray(I);
    for i=1:n
        M=bkImg(ROI.y(i):ROI.y(i)+ROI.height(i),ROI.x(i):ROI.x(i)+ROI.width(i));
        tname=strcat('Area',num2str(i));
        figure('Numbertitle','off','Name',tname);
        imhist(M);
    end
end


% --- Executes on selection change in listbox_camList.
function listbox_camList_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_camList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_camList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_camList
camDevices=getappdata(0,'camDevices');
cnum=get(hObject,'Value');
%update the vFormat listbox
vFormats=camDevices(cnum).SupportedFormats;
set(handles.listbox_vFormats,'String',vFormats);
set(handles.listbox_vFormats,'Value',1);  
%create vidobj using defaultFormat
vidobj=getappdata(0,'vidobj');
if ~isempty(vidobj)
    delete(vidobj);
end
vidobj = createVidObj(camDevices(cnum),camDevices(cnum).DefaultFormat,handles);
setappdata(0,'vidobj',vidobj);



% --- Executes during object creation, after setting all properties.
function listbox_camList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_camList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_ctrl_txtData.
function pushbutton_ctrl_txtData_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ctrl_txtData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
[FileName,PName] = uiputfile('*.txt','Save track data',PathName);
if FileName~=0
    PathName=PName;
    setappdata(0,'PathName',PathName);
    fn=strcat(PathName,FileName);
    saveTxtData(fn);
    %also save mat-file
    saveMatFile();
end

function saveTxtData(FileName)
versionName=getappdata(0,'versionName');
conditionedArea=getappdata(0,'conditionedArea');
nowTim=getappdata(0,'nowTim');
ROI=getappdata(0,'ROI');
bkImg=getappdata(0,'bkImg2D');
wh=size(bkImg);
trackData=getappdata(0,'trackData');
scoreData=getappdata(0,'scoreData');
cnum=length(scoreData);
ardInfo=getappdata(0,'ardInfo');
detectFreezingTag=getappdata(0,'detectFreezingTag');
freezingData=getappdata(0,'freezingData');       
freezingTotal=getappdata(0,'freezingTotal');          
fid=fopen(FileName,'wt');
fprintf(fid,'Code_version: %s\r\n',versionName);
if nowTim(1)>1
    rTim=floor(nowTim(1));
else
    rTim=ardInfo.totalTime;
end
fprintf(fid,'Total_recorded_time(s): %d\r\n',rTim);
%for fear conditioning 
if detectFreezingTag
    %behavioral parameters
    fprintf(fid,'Parameters used for behavioral test:\r\n');
    fprintf(fid,'delay time (sec): %d\r\n',ardInfo.delay);
    fprintf(fid,'number to repeat: %d\r\n',ardInfo.repeat);
    fprintf(fid,'interval time between trials(sec): %d\r\n',ardInfo.interval);
    fprintf(fid,'tone check: %d\r\n',ardInfo.toneFlag);
    fprintf(fid,'tone duration (sec): %d\r\n',ardInfo.toneDuration);
    fprintf(fid,'shock check: %d\r\n',ardInfo.shockFlag);
    fprintf(fid,'shock duration (sec): %d\r\n',ardInfo.shockDuration);
    %results
    fprintf(fid,'total freezing time (sec): %8.2f\r\n',freezingTotal);
    fprintf(fid,'freezing episodes(startTime/endTime/duration):\r\n');
    fprintf(fid,'%8.2f %8.2f %8.2f\r\n',freezingData');
end
%for 2-chamber place preference
% if ardInfo.laserFlag
    fprintf(fid,'Chamber_coupled_with_reinforcement: %d\r\n',conditionedArea);
    fprintf(fid,'Number_of_chambers: %d\r\n',cnum);
    fprintf(fid,'Score_for_chambers(1/2):');
    fprintf(fid,'%8.3f ',scoreData);
    fprintf(fid,'\r\n\r\n');
    fprintf(fid,'ROI_of_chambers:\r\n');
    %first: resolution of background image
    fprintf(fid,'%d %d %d %d\r\n',0,0,wh(1),wh(2));
    for i=1:cnum
        fprintf(fid,'%d %d %d %d\r\n',ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i));
    end
    fprintf(fid,'\r\n');
% end
%tracking points
fprintf(fid,'\r\nTotal_tracked_dots: %d\r\n',size(trackData,1));
% fprintf(fid,'Track_points(time/x/y):\r\n');
% fprintf(fid,'%8.2f %8.2f %8.2f\r\n',trackData');
fclose(fid);


function edit_conditionedAreaNum_Callback(hObject, eventdata, handles)
% hObject    handle to edit_conditionedAreaNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_conditionedAreaNum as text
%        str2double(get(hObject,'String')) returns contents of edit_conditionedAreaNum as a double
cnum=str2double(get(hObject,'String'));
setappdata(0,'conditionedArea',cnum);

% --- Executes during object creation, after setting all properties.
function edit_conditionedAreaNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_conditionedAreaNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_Ardu_Output_Tone.
function checkbox_Ardu_Output_Tone_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_Ardu_Output_Tone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_Ardu_Output_Tone
ardInfo=getappdata(0,'ardInfo');
ardInfo.toneFlag=get(hObject,'Value');
setappdata(0,'ardInfo',ardInfo);


function edit_Ardu_Output_ToneDuration_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_Output_ToneDuration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Ardu_Output_ToneDuration as text
%        str2double(get(hObject,'String')) returns contents of edit_Ardu_Output_ToneDuration as a double
ardInfo=getappdata(0,'ardInfo');
ardInfo.toneDuration=str2double(get(hObject,'String'));
setappdata(0,'ardInfo',ardInfo);
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    cmdStr=['t',num2str(ardInfo.toneDuration),'/'];
    fwrite(myArduino,cmdStr,'uchar');
end

% --- Executes during object creation, after setting all properties.
function edit_Ardu_Output_ToneDuration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_Output_ToneDuration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_Ardu_Output_Shock.
function checkbox_Ardu_Output_Shock_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_Ardu_Output_Shock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_Ardu_Output_Shock
ardInfo=getappdata(0,'ardInfo');
ardInfo.shockFlag=get(hObject,'Value');
setappdata(0,'ardInfo',ardInfo);


function edit_Ardu_Output_ShockDuration_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_Output_ShockDuration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Ardu_Output_ShockDuration as text
%        str2double(get(hObject,'String')) returns contents of edit_Ardu_Output_ShockDuration as a double
ardInfo=getappdata(0,'ardInfo');
ardInfo.shockDuration=str2double(get(hObject,'String'));
setappdata(0,'ardInfo',ardInfo);
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    cmdStr=['s',num2str(ardInfo.shockDuration),'/'];
    fwrite(myArduino,cmdStr,'uchar');
end
        

% --- Executes during object creation, after setting all properties.
function edit_Ardu_Output_ShockDuration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_Output_ShockDuration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_Ardu_Output_Laser.
function checkbox_Ardu_Output_Laser_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_Ardu_Output_Laser (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_Ardu_Output_Laser
ardInfo=getappdata(0,'ardInfo');
ardInfo.laserFlag=get(hObject,'Value');
setappdata(0,'ardInfo',ardInfo);


function edit_Ardu_delay_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_delay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Ardu_delay as text
%        str2double(get(hObject,'String')) returns contents of edit_Ardu_delay as a double
ardInfo=getappdata(0,'ardInfo');
ardInfo.delay=str2double(get(hObject,'String'));
setappdata(0,'ardInfo',ardInfo);

% --- Executes during object creation, after setting all properties.
function edit_Ardu_delay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_delay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Ardu_repeat_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_repeat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Ardu_repeat as text
%        str2double(get(hObject,'String')) returns contents of edit_Ardu_repeat as a double
ardInfo=getappdata(0,'ardInfo');
ardInfo.repeat=str2double(get(hObject,'String'));
setappdata(0,'ardInfo',ardInfo);

% --- Executes during object creation, after setting all properties.
function edit_Ardu_repeat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_repeat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Ardu_interval_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Ardu_interval as text
%        str2double(get(hObject,'String')) returns contents of edit_Ardu_interval as a double
ardInfo=getappdata(0,'ardInfo');
ardInfo.interval=str2double(get(hObject,'String'));
setappdata(0,'ardInfo',ardInfo);

% --- Executes during object creation, after setting all properties.
function edit_Ardu_interval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Ardu_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_detectFreezing.
function checkbox_detectFreezing_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_detectFreezing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_detectFreezing
val=get(hObject,'Value');
setappdata(0,'detectFreezingTag',val);
if val
    setappdata(0,'taskTag',2);
    trackData=getappdata(0,'trackData');
    velData=getappdata(0,'velocityData');
    if ~isempty(trackData) && ~isempty(velData)
        [freezingData,moveTag]=getFreezing(trackData,velData);
        freezingTotal=sum(freezingData(:,3));
        set(handles.text_freezingSec,'String',num2str(freezingTotal,'%8.1f'));
        setappdata(0,'freezingTotal',freezingTotal);
        setappdata(0,'freezingData',freezingData);
        setappdata(0,'moveTag',moveTag);
        disp('Freezing episodes(start/end/duration(s))');
        disp(freezingData);
    end
else
    setappdata(0,'taskTag',0);
end

%calculate the background by removing the moving objects
function I=getBackgroundImg(bkImgBuffer)
n=size(bkImgBuffer,3);
if n<2
    I=bkImgBuffer;
    return;
end
th=20;      %threshold image to detect moving objects
I0=bkImgBuffer(:,:,2);
M1=bkImgBuffer(:,:,2:end);
%randomize the order
idx=randperm(n-1);
M2=bkImgBuffer(:,:,idx);
%idx=randperm(n-1);
idx=mod(idx+1+floor(n/10),n-1)+1;
M3=bkImgBuffer(:,:,idx);
%I0=repmat(bkImgBuffer(:,:,1),[1,1,n-1]);
d1=abs(double(M1)-double(M2))<th;
d2=abs(double(M1)-double(M3))<th;
d=d1&d2;
dImg=double(bkImgBuffer(:,:,2:end)).*d;
Img1=max(dImg,[],3);
%check if cover all
bkall=max(d,[],3);
missing=bkall==0;
Img0=double(I0).*missing;
I=uint8(max(Img1,Img0));
if max(missing(:))>0
    disp('background image possibly wrong, redo it!');
else
    disp('background image generated sucessfully');
end


function edit_freezingCutoff_Callback(hObject, eventdata, handles)
% hObject    handle to edit_freezingCutoff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_freezingCutoff as text
%        str2double(get(hObject,'String')) returns contents of edit_freezingCutoff as a double
freezingPars=getappdata(0,'freezingPars');
freezingPars(2)=str2double(get(hObject,'String'));
setappdata(0,'freezingPars',freezingPars);

% --- Executes during object creation, after setting all properties.
function edit_freezingCutoff_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_freezingCutoff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_freezingVth_Callback(hObject, eventdata, handles)
% hObject    handle to edit_freezingVth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_freezingVth as text
%        str2double(get(hObject,'String')) returns contents of edit_freezingVth as a double
freezingPars=getappdata(0,'freezingPars');
freezingPars(1)=str2double(get(hObject,'String'));
setappdata(0,'freezingPars',freezingPars);

% --- Executes during object creation, after setting all properties.
function edit_freezingVth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_freezingVth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_protocol_load.
function pushbutton_protocol_load_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_protocol_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
[FileName,PName] = uigetfile('*.prot','Load protocol',PathName);
if FileName~=0
    fname=fullfile(PName,FileName);
    fid=fopen(fname);
    tmpstr=textscan(fid,'%s %d',8);
    ardInfo=getappdata(0,'ardInfo');
    %the sequence 
    ardInfo.totalTime=tmpstr{2}(1);
    ardInfo.delay=tmpstr{2}(2);
    ardInfo.repeat=tmpstr{2}(3);
    ardInfo.interval=tmpstr{2}(4);
    ardInfo.toneFlag=tmpstr{2}(5);
    ardInfo.toneDuration=tmpstr{2}(6);
    ardInfo.shockFlag=tmpstr{2}(7);
    ardInfo.shockDuration=tmpstr{2}(8);
    fclose(fid);
    setappdata(0,'ardInfo',ardInfo);
    updateProtPanel(ardInfo,handles);
    myArduino=getappdata(0,'myArduino');
    updateArduino(myArduino, ardInfo);
    setappdata(0,'recTime',ardInfo.totalTime);
    set(handles.edit_RecTime,'String',s2hhmmss(ardInfo.totalTime,0));
end



% --- Executes on button press in pushbutton_protocol_save.
function pushbutton_protocol_save_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_protocol_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
[FileName,PName] = uiputfile('*.prot','Save protocol',PathName);
if FileName~=0
    fname=fullfile(PName,FileName);
    ardInfo=getappdata(0,'ardInfo');
    fid=fopen(fname,'wt');
    fprintf(fid,'total_record_time_(sec): %d\r\n',ardInfo.totalTime);
    fprintf(fid,'delay_time_(sec): %d\r\n',ardInfo.delay);
    fprintf(fid,'number_to_repeat: %d\r\n',ardInfo.repeat);
    fprintf(fid,'interval_time_between_trials(sec): %d\r\n',ardInfo.interval);
    fprintf(fid,'tone_check: %d\r\n',ardInfo.toneFlag);
    fprintf(fid,'tone_duration_(sec): %d\r\n',ardInfo.toneDuration);
    fprintf(fid,'shock_check: %d\r\n',ardInfo.shockFlag);
    fprintf(fid,'shock_duration_(sec): %d\r\n',ardInfo.shockDuration);
    fclose(fid);
end


% --- Executes on button press in checkbox_NOR.
function checkbox_NOR_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_NOR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_NOR
if get(hObject,'Value')
    setappdata(0,'taskTag',1);
else
    setappdata(0,'taskTag',0);
end

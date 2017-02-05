function varargout = Quiz_2_1_1_Question_2(varargin)
% QUIZ_2_1_1_QUESTION_2 MATLAB code for Quiz_2_1_1_Question_2.fig
%      QUIZ_2_1_1_QUESTION_2, by itself, creates a new QUIZ_2_1_1_QUESTION_2 or raises the existing
%      singleton*.
%
%      H = QUIZ_2_1_1_QUESTION_2 returns the handle to a new QUIZ_2_1_1_QUESTION_2 or the handle to
%      the existing singleton*.
%
%      QUIZ_2_1_1_QUESTION_2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in QUIZ_2_1_1_QUESTION_2.M with the given input arguments.
%
%      QUIZ_2_1_1_QUESTION_2('Property','Value',...) creates a new QUIZ_2_1_1_QUESTION_2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Quiz_2_1_1_Question_2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Quiz_2_1_1_Question_2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Quiz_2_1_1_Question_2

% Last Modified by GUIDE v2.5 12-Mar-2016 16:34:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Quiz_2_1_1_Question_2_OpeningFcn, ...
                   'gui_OutputFcn',  @Quiz_2_1_1_Question_2_OutputFcn, ...
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


% --- Executes just before Quiz_2_1_1_Question_2 is made visible.
function Quiz_2_1_1_Question_2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Quiz_2_1_1_Question_2 (see VARARGIN)
if ~isfield(handles,'hListener')
    handles.hListener = ...
        addlistener(handles.slider3,'ContinuousValueChange',@respondToContSlide3Callback);
end

% Choose default command line output for Quiz_2_1_1_Question_2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Quiz_2_1_1_Question_2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Quiz_2_1_1_Question_2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;






function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double






% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


gain = get(handles.slider3,'Value')*.25;
SLIP_HybridSim_RaibertHopper(0.05,gain,hObject,2);
    






function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
if str2double(get(hObject,'String'))<0
    set(hObject,'String','0');
    set(handles.slider3,'Value',0)
elseif str2double(get(hObject,'String'))>0.25
    set(hObject,'String','0.25');
    set(handles.slider3,'Value',1);
elseif str2double(get(hObject,'String'))>=0 || str2double(get(hObject,'String'))<=0.25
    set(handles.slider3,'Value',str2double(get(hObject,'String'))/0.25);
else
    set(hObject,'String','0');
    set(handles.slider3,'Value',0);
end

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
    set(hObject,'Value',0);
    set(handles.edit2,'String','0');
end

function respondToContSlide3Callback(hObject,eventdata)
%establish handles object as belonging to Gui object
handles = guidata(hObject);


%sldval = get(hObject,'Value')*2*pi;
gain = get(hObject,'Value')*.25;
set(handles.edit2,'String',num2str(gain));

function varargout = BezGuiAttempt(varargin)
% BEZGUIATTEMPT MATLAB code for BezGuiAttempt.fig
%      BEZGUIATTEMPT, by itself, creates a new BEZGUIATTEMPT or raises the existing
%      singleton*.
%
%      H = BEZGUIATTEMPT returns the handle to a new BEZGUIATTEMPT or the handle to
%      the existing singleton*.
%
%      BEZGUIATTEMPT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BEZGUIATTEMPT.M with the given input arguments.
%
%      BEZGUIATTEMPT('Property','Value',...) creates a new BEZGUIATTEMPT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BezGuiAttempt_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BezGuiAttempt_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BezGuiAttempt

% Last Modified by GUIDE v2.5 21-Apr-2014 23:18:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BezGuiAttempt_OpeningFcn, ...
                   'gui_OutputFcn',  @BezGuiAttempt_OutputFcn, ...
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


% --- Executes just before BezGuiAttempt is made visible.
function BezGuiAttempt_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BezGuiAttempt (see VARARGIN)
global points;
global drag;
drag = 0;
% Choose default command line output for BezGuiAttempt
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

P0 = [2*pi/3; -4*pi/3];
P3 = [pi/3; -2*pi/3];
P1 = [1.50; -2.70];
P2 = [1.44; -2.16];
points = [P0'; P1'; P2'; P3'];
[th1, th2, h] = cubicBezier(P0,P1,P2,P3);
% set(h, 'HitTest', 'off');
set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
axes(handles.axes1)
visualiseStatic(th1, th2);
set(handles.uitable1, 'Data', points);
uiwait(handles.figure1);

% % This sets up the initial plot - only do when we are invisible
% % so window can get raised using BezGuiAttempt.
% if strcmp(get(hObject,'Visible'),'off')
%     plot(rand(5));
% end

% UIWAIT makes BezGuiAttempt wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = BezGuiAttempt_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points;
% Get default command line output from handles structure
varargout{1} = points;


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on mouse press over axes background.
function axes3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
p = get(gca, 'CurrentPoint');
p = p(1,1:2);
points(end, :) = [p(1) p(2)];
set(handles.uitable1, 'Data', points);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
uiresume(handles.figure1);
delete(handles.figure1);

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton4.
function pushbutton4_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume(handles.figure1);


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
p = get(gca, 'CurrentPoint');
p = p(1,1:2);
points(end + 1, :) = [p(1) p(2)];
set(handles.uitable1, 'Data', points);

% Executes on button press on lines within the plot
function bezplot_ButtonDownFcn(o,e)
global points
global drag
p = get(gca, 'CurrentPoint');
p = p(1,1:2);

% Search through points to see if button pressed near point
% First check x coord.
xvals = points(:,1);
possPoints = find(abs(xvals(:,1) - p(1)) < 0.1);
% Then iterate through possible x coords for match in both x and y
if ~isempty(possPoints)
    for i = possPoints'
        if abs(points(i,2) - p(2)) < 0.1
            drag = i;
            break;
        end
    end
end

% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
global drag

if (drag)
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    points(drag,:) = p;
    drag = 0;
    h = updatePlots(handles);
    set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
end


function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
global drag

if (drag)
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    points(drag,:) = p;
    updatePlots(handles);
end

function h = updatePlots(handles)
global points
set(handles.uitable1, 'Data', points);
P0 = points(1,:)';
P1 = points(2,:)';
P2 = points(3,:)';
P3 = points(4,:)';
axes(handles.axes3);
[th1, th2, h] = cubicBezier(P0,P1,P2,P3);
axes(handles.axes1)
visualiseStatic(th1, th2);


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume(handles.figure1);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
uiresume(handles.figure1);
delete(hObject);

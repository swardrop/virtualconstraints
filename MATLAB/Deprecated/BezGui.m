function varargout = BezGui(varargin)
% BEZGUI MATLAB code for BezGui.fig
%      BEZGUI, by itself, creates a new BEZGUI or raises the existing
%      singleton*.
%
%      H = BEZGUI returns the handle to a new BEZGUI or the handle to
%      the existing singleton*.
%
%      BEZGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BEZGUI.M with the given input arguments.
%
%      BEZGUI('Property','Value',...) creates a new BEZGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BezGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BezGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BezGui

% Last Modified by GUIDE v2.5 10-Jul-2014 01:06:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BezGui_OpeningFcn, ...
                   'gui_OutputFcn',  @BezGui_OutputFcn, ...
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


% --- Executes just before BezGui is made visible.
function BezGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BezGui (see VARARGIN)
global points;
global drag;
drag = 0;
% Choose default command line output for BezGui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Start and end points
points = [];
points(1,:) = [2*pi/3 -4*pi/3];
points(6,:) = [pi/3; -2*pi/3];
updatePoints; % Set appropriate th_1 vals according to endpoints
% Set th_2 values such that the constraint is initially a straight line.
points(2:end-1,2) = -2*points(2:end-1,1);

axes(handles.axes3)
[th1, th2, h] = plotBez(points);
% set(h, 'HitTest', 'off');
set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
axes(handles.axes1)
visualiseStatic(th1, th2);
set(handles.uitable1, 'Data', points');
plotGammaPsi(handles);

% % This sets up the initial plot - only do when we are invisible
% % so window can get raised using BezGui.
% if strcmp(get(hObject,'Visible'),'off')
%     plot(rand(5));
% end

% UIWAIT makes BezGui wait for user response (see UIRESUME)
uiwait(handles.figure1);
pause(1)


% --- Outputs from this function are returned to the command line.
function varargout = BezGui_OutputFcn(hObject, eventdata, handles)
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
%uiresume(handles.figure1);


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
global drag dragEnd
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
            if i == 1 || i == length(xvals)
                dragEnd = true;
            end
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
global drag dragEnd

if (drag)
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    points(drag,2) = p(2);
    drag = false;
    if (dragEnd)
        points(drag,1) = p(1);
        updatePoints
        dragEnd = false;
    end
    h = updatePlots(handles);
    set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
    plotGammaPsi(handles);
end


function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
global drag dragEnd

if (drag) && 0
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    points(drag,2) = p(2);
    if (dragEnd)
        points(drag,1) = p(1);
        updatePoints;
    end
    updatePlots(handles);
end

function h = updatePlots(handles)
global points
set(handles.uitable1, 'Data', points');
axes(handles.axes3);
[th1, th2, h] = plotBez(points);
pause(1)
axes(handles.axes1)
visualiseStatic(th1, th2);
%drawnow; pause(0.2)

% Scale control point positions based on endpoints.
function updatePoints()
global points
N = size(points,1);
% Note that x value must be i*(xf-x0)/n + x0 due to
% functional formulation of bezier curve.
for i = 2 : N-1
    x = (i-1)*(points(N,1)-points(1,1))/(N-1) + points(1,1);
    points(i,1) = x;
end

function plotGammaPsi(handles)
global points
[Gamma, Psi, theta, th_c] = PartialSolZeroDyn(points);
axes(handles.axes6)
hold off
plot(theta,Gamma)
hold on
sc_fact = 0.05*(max(Gamma)-min(Gamma));
plot([th_c th_c], [min(Gamma)-sc_fact, max(Gamma)+sc_fact], 'k-');
axes(handles.axes7)
hold off
plot(theta,Psi)
sc_fact = 0.05*(max(Psi)-min(Psi));
hold on
plot([th_c th_c], [min(Psi)-sc_fact, max(Psi)+sc_fact], 'k-');

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


% --- Executes when entered data in editable cell(s) in uitable1.
function uitable1_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable1 (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
global points;
points = get(handles.uitable1, 'Data')';
updatePoints;
updatePlots(handles);
plotGammaPsi(handles);

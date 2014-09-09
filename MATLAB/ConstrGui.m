function varargout = ConstrGui(varargin)
% CONSTRGUI MATLAB code for ConstrGui.fig
%      CONSTRGUI, by itself, creates a new CONSTRGUI or raises the existing
%      singleton*.
%
%      H = CONSTRGUI returns the handle to a new CONSTRGUI or the handle to
%      the existing singleton*.
%
%      CONSTRGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONSTRGUI.M with the given input arguments.
%
%      CONSTRGUI('Property','Value',...) creates a new CONSTRGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ConstrGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ConstrGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ConstrGui

% Last Modified by GUIDE v2.5 13-Aug-2014 21:27:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ConstrGui_OpeningFcn, ...
                   'gui_OutputFcn',  @ConstrGui_OutputFcn, ...
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


% --- Executes just before ConstrGui is made visible.
function ConstrGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ConstrGui (see VARARGIN)
global points origPoints th1 th2;
global constrData;
global drag;
drag = 0;

% Start and end points
points = [];
points(1,:) = [7*pi/12 -7*pi/6];
points(6,:) = [5*pi/12; -5*pi/6];
updatePoints; % Set appropriate th_1 vals according to endpoints
% Set th_2 values such that the constraint is initially a straight line.
points(2:end-1,2) = -2*points(2:end-1,1);
origPoints = points;

constrData = makeConstr(points);
% Choose default command line output for ConstrGui
handles.output = constrData;

% Update handles structure
guidata(hObject, handles);

axes(handles.axes3)
[th1, th2, h] = plotBez(points);
set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
set(handles.uitable1, 'Data', points');
refreshSidePlots(handles);

% UIWAIT makes ConstrGui wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ConstrGui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constrData;
% Get default command line output from handles structure
varargout{1} = constrData(2:end);


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
        if abs(points(i,2) - p(2)) < 0.05
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
    h = refreshBezDisplay(handles);
    set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
    refreshSidePlots(handles);
end


function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points
global drag dragEnd

if (drag)
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    points(drag,2) = p(2);
    if (dragEnd)
        points(drag,1) = p(1);
        updatePoints;
    end
    refreshBezDisplay(handles);
end

function h = refreshBezDisplay(handles)
global points th1 th2
set(handles.uitable1, 'Data', points');
axes(handles.axes3);
[th1, th2, h] = plotBez(points);

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

function refreshSidePlots(handles)
global points th1 th2

axes(handles.axes1)
visualiseStatic(th1, th2);

[Gamma, Psi, theta, th_c] = PartialSolZeroDyn(points);
axes(handles.axes6)
hold off
plot(theta,Gamma)
hold on
sc_fact = 0.05*(max(Gamma)-min(Gamma));
plot([th_c th_c], [min(Gamma)-sc_fact, max(Gamma)+sc_fact], 'k-');
ylim([min(Gamma)-sc_fact, max(Gamma)+sc_fact])
grid on
axes(handles.axes7)
hold off
plot(theta,Psi)
sc_fact = 0.05*(max(Psi)-min(Psi));
hold on
plot([th_c th_c], [min(Psi)-sc_fact, max(Psi)+sc_fact], 'k-');
ylim([min(Psi)-sc_fact, max(Psi)+sc_fact]);
grid on

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
refreshBezDisplay(handles);
refreshSidePlots(handles);


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constrData points

% Check that the points are unique
for i = 2:length(constrData)
    if points == constrData(i).points
        set(handles.numConstrSaved, 'String', ...
            [num2str(length(constrData) - 1) ' constraints saved ' ...
            '- Duplicate constraint not saved']);
        return
    end
end
constrData(end+1) = makeConstr(points);
set(handles.numConstrSaved, 'String', [num2str(length(constrData) - 1) ...
    ' constraints saved']);

% --- Executes on button press in resetButton.
function resetButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global points origPoints
points = origPoints;
h = refreshBezDisplay(handles);
set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
refreshSidePlots(handles);


% --- Executes on button press in orderingsButton.
function orderingsButton_Callback(hObject, eventdata, handles)
% hObject    handle to orderingsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constrData
if length(constrData) > 1
    orderings(constrData(2:end))
else
    disp('Cannot open window for fewer than two constraints');
end

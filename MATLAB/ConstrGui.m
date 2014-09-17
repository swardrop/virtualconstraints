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

% Last Modified by GUIDE v2.5 14-Sep-2014 20:53:09

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
global constr orig_th orig_al theta q;
global constrData;
global drag;
global type;
drag = 0;

N = 5;

% Start and end points
constr = struct;
constr.theta_p(1) = -pi/12;
constr.theta_p(N) = pi/12;
constr.alpha_p(:,1) = -pi/6;
constr.alpha_p(:,N) = pi/6;
updatePoints; % Set appropriate th_1 vals according to endpoints
% Set th_2 values such that the constraint is initially a straight line.
constr.alpha_p(:,2:end-1) = 2*constr.theta_p(2:end-1);
updatePoints; % Impose invariance condition
orig_th = constr.theta_p;
orig_al = constr.alpha_p;
constr = makeConstr(constr.theta_p, constr.alpha_p);

constrData = constr;
% Choose default command line output for ConstrGui
handles.output = constrData;

% Update handles structure
guidata(hObject, handles);

axes(handles.axes3)
[h, theta, q] = plotBez(constr.theta_p, constr.alpha_p);
set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
set(handles.uitable1, 'Data', [constr.alpha_p; constr.theta_p]);
refreshSidePlots(handles);

contents = cellstr(get(handles.costmenu,'String'));
type = contents{get(handles.costmenu,'Value')};

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

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constrData
if length(constrData) < 2
    saveButton_Callback(hObject, eventdata, handles)
end
uiresume(handles.figure1);
delete(handles.figure1);

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton4.
function pushbutton4_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%uiresume(handles.figure1);

% Executes on button press on lines within the plot
function bezplot_ButtonDownFcn(o,e)
global constr
global drag dragEnd
p = get(gca, 'CurrentPoint');
p = p(1,1:2);

theta_p = constr.theta_p;
alpha_p = constr.alpha_p;
% Search through points to see if button pressed near point
% First check theta coord.
possPoints = find(abs(theta_p - p(1)) < 0.1);
% Then iterate through possible th coords for match in both th and q
for i = possPoints
    if abs(alpha_p(:,i) - p(2)) < 0.05
        drag = i;
        if i == 1 || i == 2 ...
                  || i == length(theta_p)-1 || i == length(theta_p)
            dragEnd = true;
        end
        break;
    end
end

% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constr
global drag dragEnd

if (drag)
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    constr.alpha_p(:,drag) = p(2);
    drag = false;
    if (dragEnd)
        constr.theta_p(drag) = p(1);
        updatePoints
        dragEnd = false;
    end
    h = refreshBezDisplay(handles);
    set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
    constr = makeConstr(constr.theta_p, constr.alpha_p);
    refreshSidePlots(handles);
end


function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constr
global drag dragEnd

if (drag)
    p = get(handles.axes3, 'CurrentPoint');
    p = p(1,1:2);
    constr.alpha_p(:,drag) = p(2);
    if (dragEnd)
        constr.theta_p(drag) = p(1);
        updatePoints;
    end
    refreshBezDisplay(handles);
end

function h = refreshBezDisplay(handles)
global constr theta q
set(handles.uitable1, 'Data', [constr.alpha_p; constr.theta_p]);
axes(handles.axes3);
[h, theta, q] = plotBez(constr.theta_p, constr.alpha_p);

% Scale control point positions based on endpoints.
function updatePoints()
global constr
N = length(constr.theta_p);

% Impose self-invariance
[p0, th0, p1] = invarianceCond(...
    constr.theta_p, constr.alpha_p, constr.theta_p(end), N, true);
constr.theta_p(1) = th0;
constr.alpha_p(:,1) = p0;
constr.alpha_p(:,2) = p1;

% Note that x value must be i*(xf-x0)/n + x0 due to
% functional formulation of bezier curve.
for i = 2 : N-1
    constr.theta_p(i) = (i-1)*(constr.theta_p(N)-constr.theta_p(1))/(N-1)...
        + constr.theta_p(1);
end

function refreshSidePlots(handles)
global constr q

axes(handles.axes1)
visualiseStatic(q);

th_base = constr.th_base; Gamma = constr.Gamma; Psi = constr.Psi;
th_c = constr.theta_c;
axes(handles.axes6)
hold off
plot(th_base,Gamma)
hold on
sc_fact = 0.05*(max(Gamma)-min(Gamma));
plot([th_c th_c], [min(Gamma)-sc_fact, max(Gamma)+sc_fact], 'k-');
ylim([min(Gamma)-sc_fact, max(Gamma)+sc_fact])
grid on
axes(handles.axes7)
hold off
plot(th_base,Psi)
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
global constr;
points = get(handles.uitable1, 'Data');
constr.alpha_p = points(1:end-1,:);
constr.theta_p = points(end,:);
updatePoints;
constr = makeConstr(constr.theta_p, constr.alpha_p);
h = refreshBezDisplay(handles);
set(h, 'ButtonDownFcn', @bezplot_ButtonDownFcn);
refreshSidePlots(handles);


% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constrData constr

% Check that the points are unique
for i = 2:length(constrData)
    if all(constr.alpha_p == constrData(i).alpha_p) ...
            && all(constr.theta_p == constrData(i).theta_p)
        set(handles.numConstrSaved, 'String', ...
            [num2str(length(constrData) - 1) ' constraints saved ' ...
            '- Duplicate constraint not saved']);
        return
    end
end
constrData(end+1) = constr;
set(handles.numConstrSaved, 'String', [num2str(length(constrData) - 1) ...
    ' constraints saved']);

% --- Executes on button press in resetButton.
function resetButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constr orig_th orig_al
constr.theta_p = orig_th;
constr.alpha_p = orig_al;
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


% --- Executes on button press in gridbutton.
function gridbutton_Callback(hObject, eventdata, handles)
% hObject    handle to gridbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global constr type

N = str2double(get(handles.editN, 'String'));
sqthetadot_0 = str2double(get(handles.editvel, 'String'));

gridTorques(constr.theta_p(end), constr.alpha_p(:,end), ...
    N, sqthetadot_0, type);


function editN_Callback(hObject, eventdata, handles)
% hObject    handle to editN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editN as text
%        str2double(get(hObject,'String')) returns contents of editN as a double


% --- Executes during object creation, after setting all properties.
function editN_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editvel_Callback(hObject, eventdata, handles)
% hObject    handle to editvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editvel as text
%        str2double(get(hObject,'String')) returns contents of editvel as a double


% --- Executes during object creation, after setting all properties.
function editvel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in costmenu.
function costmenu_Callback(hObject, eventdata, handles)
% hObject    handle to costmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns costmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from costmenu

global type
contents = cellstr(get(hObject,'String'));
type = contents{get(hObject,'Value')};

% --- Executes during object creation, after setting all properties.
function costmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to costmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

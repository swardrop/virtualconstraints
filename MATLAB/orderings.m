function varargout = orderings(varargin)
% ORDERINGS MATLAB code for orderings.fig
%      ORDERINGS, by itself, creates a new ORDERINGS or raises the existing
%      singleton*.
%
%      H = ORDERINGS returns the handle to a new ORDERINGS or the handle to
%      the existing singleton*.
%
%      ORDERINGS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ORDERINGS.M with the given input arguments.
%
%      ORDERINGS('Property','Value',...) creates a new ORDERINGS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before orderings_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to orderings_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help orderings

% Last Modified by GUIDE v2.5 13-Aug-2014 20:51:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @orderings_OpeningFcn, ...
                   'gui_OutputFcn',  @orderings_OutputFcn, ...
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


% --- Executes just before orderings is made visible.
function orderings_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to orderings (see VARARGIN)
global Gamma_cs Psi_cs

constrData = varargin{1};
Gamma_cs = [constrData.Gamma_c];
Psi_cs = [constrData.Psi_c];
updateDisplay(handles);

% Choose default command line output for orderings
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes orderings wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = orderings_OutputFcn(hObject, eventdata, handles) 
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
updateDisplay(handles);

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function text2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Updates the display, showing the ordering subject to the value of the
% inital velocity squared, as given by the user
function updateDisplay(handles)
global Gamma_cs Psi_cs

% Find ordering of constraints based upon given thetadot_0^2
sqthetadot_0 = str2double(get(handles.edit1, 'String'));
thetadot_cs = Gamma_cs*sqthetadot_0 + Psi_cs;
[~,ind] = sort(thetadot_cs);

axes(handles.axes1);
hold off
plot(Gamma_cs(ind(2:end)), Psi_cs(ind(2:end)), 'k-');
hold on
plot(Gamma_cs(ind(1:2)), Psi_cs(ind(1:2)));
plot(Gamma_cs, Psi_cs, 'rx');
plot(Gamma_cs(end), Psi_cs(end), 'kx'); % Last constraint added
plot(Gamma_cs(ind(1)), Psi_cs(ind(1)), 'bx'); % First in order
xlabel('\Gamma(\theta_c)'); ylabel('\Psi(\theta_c)');


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(handles.figure1)
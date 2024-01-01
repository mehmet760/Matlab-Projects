function varargout = image_processing_project(varargin)
% IMAGE_PROCESSING_PROJECT MATLAB code for image_processing_project.fig
%      IMAGE_PROCESSING_PROJECT, by itself, creates a new IMAGE_PROCESSING_PROJECT or raises the existing
%      singleton*.
%
%      H = IMAGE_PROCESSING_PROJECT returns the handle to a new IMAGE_PROCESSING_PROJECT or the handle to
%      the existing singleton*.
%
%      IMAGE_PROCESSING_PROJECT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMAGE_PROCESSING_PROJECT.M with the given input arguments.
%
%      IMAGE_PROCESSING_PROJECT('Property','Value',...) creates a new IMAGE_PROCESSING_PROJECT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before image_processing_project_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to image_processing_project_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help image_processing_project

% Last Modified by GUIDE v2.5 24-Dec-2023 13:51:12

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @image_processing_project_OpeningFcn, ...
                   'gui_OutputFcn',  @image_processing_project_OutputFcn, ...
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


% --- Executes just before image_processing_project is made visible.
function image_processing_project_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to image_processing_project (see VARARGIN)

% Choose default command line output for image_processing_project
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes image_processing_project wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = image_processing_project_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
% hObject    handle to load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image
[filename pathname] = uigetfile({'*.jpg'},'file selector');
fullpathname = strcat(pathname,filename);
axes(handles.axes1);
image = imread(fullpathname);
imshow(image);

% --- Executes on button press in filter.
function filter_Callback(hObject, eventdata, handles)% Laplace filter
% hObject    handle to filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global image

axes(handles.axes2);
% Determine the Laplace filter 
kernel = [0 1 0; 1 -4 1; 0 1 0];

% Filter each color channel separately
filtered_r = conv2(double(image(:,:,1)), kernel, 'same');
filtered_g = conv2(double(image(:,:,2)), kernel, 'same');
filtered_b = conv2(double(image(:,:,3)), kernel, 'same');
% Filtrelenmiş görüntüyü birleştirme
filtered_image = cat(3, uint8(filtered_r), uint8(filtered_g), uint8(filtered_b));
imshow(filtered_image);

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

global image
axes(handles.axes2);

val = get(handles.slider1,'Value')+0.5;
gray_image = rgb2gray(image);
brightness = val*gray_image;
imshow(brightness);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light filter background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image
axes(handles.axes2);
imshow(image);




% --- Executes on button press in filter.
function sharp_Callback(hObject, eventdata, handles)
% hObject    handle to filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image

axes(handles.axes2);
% kernel matrix for sharpening
sharpeningKernel = [0 -1 0; -1 5 -1 ; 0 -1 0];
%[-1 -1 -1; -1 9 -1; -1 -1 -1];

% Convolution process
sharpenedImage = convn(double(image), sharpeningKernel, 'same');


sharpenedImage = uint8(sharpenedImage);
imshow(sharpenedImage);


% --- Executes on button press in Blurring.
function Blurring_Callback(hObject, eventdata, handles)
% hObject    handle to Blurring (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image

axes(handles.axes2);

sigma = 1.5; % Varyans
size = 5; 
[X, Y] = meshgrid(-size:size, -size:size);
kernel = exp(-(X.^2 + Y.^2) / (2*sigma^2)) /(2*pi*sigma^2);

% Convolution process
blurred_image = convn(double(image), kernel, 'same');

% Turns to uint8 type
blurred_image = uint8(blurred_image);
imshow(blurred_image);


% --- Executes on button press in edge.
function edge_Callback(hObject, eventdata, handles)
% hObject    handle to edge (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global image

axes(handles.axes2);
Gx = [-1 0 1; -1 0 1; -1 0 1];  % Matrix to detect vertical edges
Gy = [-1 -1 -1; 0 0 0; 1 1 1];  % Matrix to detect horizontal edges

% Kenar tespiti
edge_karel = sqrt(conv2(double(image(:,:,1)), double(Gx), 'same').^2 + conv2(double(image(:,:,1)), double(Gy), 'same').^2);
edge_karel = uint8(edge_karel);
imshow(edge_karel);


% --- Executes on button press in gray.
function gray_Callback(hObject, eventdata, handles)
% hObject    handle to gray (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image
axes(handles.axes2);
[rows, cols, ~] = size(image);
    gray_image = zeros(rows, cols);


 kernelMatrix = [0.2989, 0.5870, 0.1140];

    for i = 1:rows
        for j = 1:cols
            % Calculate the weighted sum for each channel
            gray_pixel_value = 0;
            for k = 1:3
                gray_pixel_value = gray_pixel_value + kernelMatrix(k) * image(i, j, k);
            end
            gray_image(i, j) = gray_pixel_value;
        end
    end

    % Convert back to uint8 format for display
    gray_image = uint8(gray_image);
imshow(gray_image);
%gray_kernel = 1/3 * ones(5);
%gray_image = convn(image, gray_kernel, 'same');
%imshow(gray_image);

% Display the grayscale image


% --- Executes on button press in sobel.


% --- Executes on button press in binary.
function binary_Callback(hObject, eventdata, handles)
% hObject    handle to binary (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image
axes(handles.axes2);

kernel = [1 1 1; 1 1 1; 1 1 1] / 9;

% Apply convolution
binary_img = conv2(double(image(:,:,1)), kernel, 'same');
binary_img = uint8(binary_img);

threshold = 128;
binary_imggg = binary_img > threshold;
%An evaluation is made based on values between 1 and 255 pixels. 
%If a pixel's value is greater than 128, that pixel will be white; If it is less than 128 it will be black.

imshow(binary_imggg);

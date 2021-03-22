%%
images = imageDatastore(".");
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files(1:4)); 


%% Prepare data set for camera calibration
%returns an M-by-2 matrix containing M [x, y] corner coordinates for the squares on a checkerboard. 
%The point [0,0] corresponds to the lower-right corner of the top-left square of the board.
squareSizeInMM = 29;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);

%% Perform camera calibration
% params contains the internal (intrinsic) and external (extrinsic) camera
% parameter
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints,'ImageSize',imageSize);

% figure; 
I = imread(string(images.Files(1)));
% imshow(I); 
% hold on;
% plot(imagePoints(:,1,1), imagePoints(:,2,1),'go');
% plot(params.ReprojectedPoints(:,1,1),params.ReprojectedPoints(:,2,1),'r+');
% legend('Detected Points','ReprojectedPoints');
% hold off;

J1 = undistortImage(I,params);
% figure; imshowpair(I,J1,'montage');
% title('Original Image (left) vs. Corrected Image (right)');

%% BEV - I must tune Height and Pitch
focalLength = params.Intrinsics.FocalLength; 
principalPoint = params.Intrinsics.PrincipalPoint; 
imageSize = params.ImageSize; 
camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
height = 2.1; % meters 
pitch = -0.7; % degree pitch toward the ground


sensor = monoCamera(camIntrinsics,height,'Pitch',pitch);

distAhead = 35; % 30
spaceToOneSide = 4; % 6
bottomOffset = 9; % 3
outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];

outImageSize = [NaN,250];
birdsEye = birdsEyeView(sensor,outView,outImageSize);
I = imread(string(images.Files(5))); % 5:8
BEV = transformImage(birdsEye,I);
% figure; imshow(I);
%     figure; imshow(BEV)

%% Grayscale conversion and noise reduction
BEV_grayscale = rgb2gray(BEV);
BEV_grayscale_blur = imgaussfilt(BEV_grayscale);
%     figure; imshowpair(BEV_grayscale, BEV_grayscale_blur,"montage");

%% Image binarization

% BEV_binarized = imbinarize(BEV_grayscale_blur, 'adaptive');
BEV_binarized = imbinarize(BEV_grayscale_blur)
% figure; imshow(BEV_binarized)
% title(sprintf('Pitch: %0.1f', pitch))


%% Solution

y_axis = sum(BEV_binarized);
% figure
% plot(y_axis)
% title(sprintf('Lane detection - Height: %0.1f', height))
% xlabel("Pixel positions")
% ylabel("Counts")

%% output
first_half = y_axis(1:length(y_axis)/2);
second_half = y_axis((length(y_axis)/2)+1:end);

[max1, ind1] = max(first_half);
[max2, ind2] = max(second_half);

figure; imshow(BEV)
hold on
xline(ind1,"g")
xline(ind2+length(y_axis)/2,"g")
hold off







close all

%% Prepare data set for camera calibration
images = imageDatastore(".");
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files(1:4));
squareSizeInMM = 29;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);

%% Perform camera calibration
% params contains the internal (intrinsic) and external (extrinsic) camera parameter
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints,'ImageSize',imageSize);

%% BEV - I must tune Height and Pitch
focalLength = params.Intrinsics.FocalLength; 
principalPoint = params.Intrinsics.PrincipalPoint; 
imageSize = params.ImageSize; 
camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
% n = 2;
% height = solutions(n).height; % meters 
% pitch = solutions(n).pitch; 
% imageNumber = 8; % 5:8
height = 1.76;
pitch = -0.9;
distAhead = 30; 
spaceToOneSide = 4; 
bottomOffset = 9; 
outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];
outImageSize = [NaN,250];

for iN=5:8
    sensor = monoCamera(camIntrinsics,height,'Pitch',pitch);    
    birdsEye = birdsEyeView(sensor,outView,outImageSize);
    image = imread(string(images.Files(iN))); % 5:8
    image_undistored = undistortImage(image,params);
    BEV = transformImage(birdsEye,image_undistored);        

    %Grayscale conversion and noise reduction
    BEV_grayscale = rgb2gray(BEV);
    BEV_grayscale_blur = imgaussfilt(BEV_grayscale);

    %Image binarization
    BEV_binarized = imbinarize(BEV_grayscale_blur);

    % Find out two maximum
    y_axis = sum(BEV_binarized);
    first_half = y_axis(1:length(y_axis)/2);
    second_half = y_axis((length(y_axis)/2)+1:end);
    [max1, ind1] = max(first_half);
    [max2, ind2] = max(second_half);
    
    %Plotting
    figure(1)
    subplot(1,4,iN-4); 
    imshow(BEV)
    hold on
    xline(ind1,"g")
    xline(ind2+length(y_axis)/2,"g")
    hold off
    
    BEV_LowLeftLanePoint = [ind1 650];
    BEV_LowRightLanePoint = [ind2+length(y_axis)/2, 650];
    BEV_HightLeftLanePoint = [ind1 5];
    BEV_HightRightLanePoint = [ind2+length(y_axis)/2, 5];
    origin_LowLeftLanePoint = vehicleToImage(sensor, imageToVehicle(birdsEye, BEV_LowLeftLanePoint));
    origin_LowRightLanePoint = vehicleToImage(sensor, imageToVehicle(birdsEye, BEV_LowRightLanePoint));
    origin_HightLeftLanePoint = vehicleToImage(sensor, imageToVehicle(birdsEye, BEV_HightLeftLanePoint));
    origin_HightRightLanePoint = vehicleToImage(sensor, imageToVehicle(birdsEye, BEV_HightRightLanePoint));

    figure(2)
    subplot(2,2,iN-4)
    imshow(image_undistored)
    hold on
    x=[origin_LowLeftLanePoint(1) origin_HightLeftLanePoint(1)];
    y=[origin_LowLeftLanePoint(2) origin_HightLeftLanePoint(2)];
    line(x,y, "Color","green", "LineWidth",2)
    x=[origin_LowRightLanePoint(1) origin_HightRightLanePoint(1)];
    y=[origin_LowRightLanePoint(2) origin_HightRightLanePoint(2)];
    line(x,y, "Color","green", "LineWidth",2)
    hold off
end

%% GridSearch
% I = imread(string(images.Files(5))); % 5:8
% min_std = +inf;
% final_height = 0;
% final_pitch = 0;
% count = 1;
% count2 = 1;
% clear solutions
% 
% focalLength = params.Intrinsics.FocalLength; 
% principalPoint = params.Intrinsics.PrincipalPoint; 
% imageSize = params.ImageSize; 
% camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
% distAhead = 30; % 35
% spaceToOneSide = 5; % 4
% bottomOffset = 9; % 9
% outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];
% outImageSize = [NaN,250];
% 
% % 1600 cycles
% for height=1.6:0.005:1.8 % 40
%     for pitch=-1:0.005:-0.8 % 40
%         sensor = monoCamera(camIntrinsics,height,'Pitch',pitch);    
%         birdsEye = birdsEyeView(sensor,outView,outImageSize);        
%         BEV = transformImage(birdsEye,I);
%         
%         BEV_grayscale = rgb2gray(BEV);
%         BEV_grayscale_blur = imgaussfilt(BEV_grayscale);
%         BEV_binarized = imbinarize(BEV_grayscale_blur);
%         
%         y_axis = sum(BEV_binarized);
%         y_axis_normalized = normalize(y_axis);
%         first_half = y_axis_normalized(1:length(y_axis_normalized)/2);
%         second_half = y_axis_normalized((length(y_axis_normalized)/2)+1:end);
%         
%         actual_std_sum = std(first_half) + std(second_half);
%         if actual_std_sum < min_std
%            min_std = actual_std_sum;
%            
%            solutions(count).std = actual_std_sum;
%            solutions(count).height = height;
%            solutions(count).pitch = pitch;
%            
%            count = count + 1;
%         end       
%         count2 = count2 + 1;
%     end
% end
% count
% count2






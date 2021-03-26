%% Prepare data set for camera calibration
%returns an M-by-2 matrix containing M [x, y] corner coordinates for the squares on a checkerboard. 
%The point [0,0] corresponds to the lower-right corner of the top-left square of the board.
images = imageDatastore(".");
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files(1:4));
squareSizeInMM = 29;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);

%% Perform camera calibration
% params contains the internal (intrinsic) and external (extrinsic) camera parameter
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints,'ImageSize',imageSize);

I = imread(string(images.Files(1)));
J1 = undistortImage(I,params);

%% BEV - I must tune Height and Pitch
% height=1.8 pitch=-0.7 distAhead = 35 spaceToOneSide = 4 bottomOffset=9
focalLength = params.Intrinsics.FocalLength; 
principalPoint = params.Intrinsics.PrincipalPoint; 
imageSize = params.ImageSize; 
camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
% n = 2;
% height = solutions(n).height; % meters 
% pitch = solutions(n).pitch; % -0.7, -1.5 degree pitch toward the ground
imageNumber = 8; % 5:8
height = 1.76;
pitch = -0.9;
distAhead = 30; % 35
spaceToOneSide = 4; % 4
bottomOffset = 9; % 9
outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];
outImageSize = [NaN,250];

figure;
for iN=5:8
    sensor = monoCamera(camIntrinsics,height,'Pitch',pitch);    
    birdsEye = birdsEyeView(sensor,outView,outImageSize);
    I = imread(string(images.Files(iN))); % 5:8
    BEV = transformImage(birdsEye,I);        
    subplot(3,4,iN-4); 
    imshow(BEV)
    title(sprintf('Image: %d, height: %0.2f, pitch: %0.1f',iN-4, height, pitch))

    %Grayscale conversion and noise reduction
    BEV_grayscale = rgb2gray(BEV);
    BEV_grayscale_blur = imgaussfilt(BEV_grayscale);
    % figure; imshowpair(BEV_grayscale, BEV_grayscale_blur,"montage");

    %Image binarization
    % BEV_binarized = imbinarize(BEV_grayscale_blur, 'adaptive');
    BEV_binarized = imbinarize(BEV_grayscale_blur);
    subplot(3,4,iN-4+4)
    imshow(BEV_binarized)

    %Solution
    y_axis = sum(BEV_binarized);
%     subplot(4,4, iN-4+8)
%     plot(y_axis)
    % title(sprintf('Lane detection - Height: %0.1f', height))
    % xlabel("Pixel positions")
    % ylabel("Counts")

    % Output
    first_half = y_axis(1:length(y_axis)/2);
    second_half = y_axis((length(y_axis)/2)+1:end);

    [max1, ind1] = max(first_half);
    [max2, ind2] = max(second_half);

    subplot(3,4,iN-4+8); 
    imshow(BEV)
    hold on
    xline(ind1,"g")
    xline(ind2+length(y_axis)/2,"g")
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

%     hold on
%     xline(43+125,"g")
%     xline(62,"g")
%     hold off





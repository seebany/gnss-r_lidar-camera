% function [h1,vecr_PCfromL0_xyzL] = get_surface_depth_from_lidar(t_datenum) used to reconstruct 
% lake surface using camera video and Lidar
% point cloud data
% -----------------------------------------------
% Written by Shahrukh Khan
% Last edit: 20 Sep 2020  (by Shahrukh Khan)
% Modified by Seebany Datta-Barua
% 23 Nov 2022: modifying for use on MATLAB_R2022b on local machine.
% 10 Dec 2022: Removing dependency on Statistics and Machine Learning
%   Toolbox, and converting to Reconstruct.m to function separate from
%   main_lidar_camera_processing.m, to process multiple cameras.
% 11 Dec 2022: merging in elements of Roohollah's mean red value
% calculation.
% 14 Dec 2022: splitting off lidar processing for surface height from the
% camera processing.
% -----------------------------------------------
%
% Data required:
% 	- GNSSR lidar point cloud data.
%
% Toolbox required:
% 	- Image Processing Toolbox.
% 	- Computer Vision Toolbox.
%
% -----------------------------------------------
%
function [h1,vecr_PCfromL0_xyzL, BoomHead, psi] = get_surface_depth_from_lidar(t_datenum, vecr_L0fromR0_tbu, debugFlag)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Load Initial Files/Values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Surface Condition (Check what is visible in camera feed)
% 1 = Full Ice
% 2 = Full Water
% 3 = Mixed Ice and Water
SurfCond = 3;

% Lidar setting
psi(1) = -45; % deg; Lidar and camera 1 elevation angle
% Boom Heading angle
% Direction the Boom is pointing (angle wrt North)
BoomHead = 70;%-300; % mu angle

%-------------rotation angles ---------------------------------------------
% mu=70;  %or 350          % check this based on the rotation   in doroste
% psi=-45;
% kapa=-45;

% Load lidar data files
if SurfCond == 2
    % If water surface condition
    % Load Pseudo Lidar points
    load('Test5_AggPC.mat'); % MAT file generated from Lid_Agg.m
    TotalLidarFrames = 12000; % 20 min of data collection gives 12000 frames
    
else
    % If ice surface condition or Ice & water mixed surface condition    
    % Lidar data file name
    pointcloud = velodyneFileReader('lidar_LM.pcap','VLP16');%'DataCamp7.pcap','VLP16'); % Enter lidar Pcap file name
    TotalLidarFrames = pointcloud.NumberOfFrames; % Obtain total no. of lidar frames
    
end

% Function to return h1 at each time.
% Here we tie the frames to be projected to the epochs that calculations
% are desired, as given by t_datenum, from the calling script.
% Assume that t_datenum spans from first frame to last frame, inclusive.
% These are all assumed to be 20-minute data collection intervals.
num_epochs = numel(t_datenum);
LidFrameNumber = 1;
% LidarFrameDiv = 11999; % No. of lidar frames to skip after each iteration (300)
% DivideFrames = TotalLidarFrames/LidarFrameDiv; % Number of parts total lidar frames are divided into
DivideFrames = num_epochs;% - 1;
LidarFrameDiv = 600; %floor(TotalLidarFrames/DivideFrames);%
% Lidar scan rate appears to be 600 scans per min.

% Loop through each lidar point cloud
LidFrameNumber = 1:LidarFrameDiv:TotalLidarFrames;
for lidar_iter = 1:numel(LidFrameNumber)-1

    if debugFlag==1 % Display text if Debugging Mode is enabled
        disp(' > Extracting lidar point cloud data ...')
    end

    % Extract the lidar point cloud for that epoch.
    pcframe = readFrame(pointcloud,LidFrameNumber(lidar_iter));
    vecr_PCfromL0_xyzL{lidar_iter} = read_point_cloud(pcframe, SurfCond);
    % Rotate the point cloud from lidar origin from lidar coords to boom
    % coordinates.
    vecr_PCfromL0_tbu = rot_tbu_from_xyzL(vecr_PCfromL0_xyzL{lidar_iter}, psi(1));
    % Translate the point cloud from lidar origin to reflected antenna
    % origin, in boom coordinates.
    vecr_PCfromL0_tbu = vecr_PCfromL0_tbu + vecr_L0fromR0_tbu;
    % Rotate from boom coords to enu coords with reflected antenna origin.
    vecr_PCfromR0_enu = rot_enu_from_tbu(vecr_PCfromL0_tbu, BoomHead);
    % Crop the point cloud to keep points more than 2 m below R0
    % and within 10 m north and south of R0.
    croprows = find(vecr_PCfromR0_enu(:,3)< -2 & ...
        vecr_PCfromR0_enu(:,1) > 0 & vecr_PCfromR0_enu(:,1) < 10 & ...
        abs(vecr_PCfromR0_enu(:,2)) < 10);
    vecr_PCfromR0_cropped_enu{lidar_iter} = vecr_PCfromR0_enu(croprows,:);
    h1(lidar_iter,1) = -mean(vecr_PCfromR0_cropped_enu{lidar_iter}(:,3));
end


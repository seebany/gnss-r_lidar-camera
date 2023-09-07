% function [h_surface_from_lidar, enu] = lidar_camera_processing.m used to reconstruct lake surface using camera video and Lidar
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
% -----------------------------------------------
%
% Data required:
%   - GNSSR camera video data.
% 	- GNSSR lidar point cloud data.
%
% Toolbox required:
% 	- Image Processing Toolbox.
% 	- Computer Vision Toolbox.
%
% Scripts required:
% 	- cam2enu.m funtion for backward projection.
% 	- Lid_Agg.m script to obtain the MAT file containing the pseudo lidar
%       point cloud, if dealing with water only surface condition
%   - myrangesearch.m function for finding points within proximity of given
%       points.
%
% -----------------------------------------------
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Load Initial Files/Values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Surface Condition (Check what is visible in camera feed)
% 1 = Full Ice
% 2 = Full Water
% 3 = Mixed Ice and Water
SurfCond = 3;

% % Lidar setting
% psi(1) = -45; % deg; Lidar and camera 1 elevation angle
% % Boom Heading angle
% % Direction the Boom is pointing (angle wrt North)
% BoomHead = 70;%-300; % mu angle
% 
% %-------------rotation angles ---------------------------------------------
% % mu=70;  %or 350          % check this based on the rotation   in doroste
% % psi=-45;
% % kapa=-45;
% 
% % Load lidar data files
% if SurfCond == 2
%     % If water surface condition
%     % Load Pseudo Lidar points
%     load('Test5_AggPC.mat'); % MAT file generated from Lid_Agg.m
%     TotalLidarFrames = 12000; % 20 min of data collection gives 12000 frames
%     
% else
%     % If ice surface condition or Ice & water mixed surface condition    
%     % Lidar data file name
%     pointcloud = velodyneFileReader('lidar_LM.pcap','VLP16');%'DataCamp7.pcap','VLP16'); % Enter lidar Pcap file name
%     TotalLidarFrames = pointcloud.NumberOfFrames; % Obtain total no. of lidar frames
%     
% end
% 
% % Function to return h1 at each time.
% % Here we tie the frames to be projected to the epochs that calculations
% % are desired, as given by t_datenum, from the calling script.
% % Assume that t_datenum spans from first frame to last frame, inclusive.
% % These are all assumed to be 20-minute data collection intervals.
num_epochs = numel(t_datenum);
% LidFrameNumber = 1;
% % LidarFrameDiv = 11999; % No. of lidar frames to skip after each iteration (300)
% % DivideFrames = TotalLidarFrames/LidarFrameDiv; % Number of parts total lidar frames are divided into
DivideFrames = num_epochs - 1;
% LidarFrameDiv = floor(TotalLidarFrames/DivideFrames);%600; %
% % Lidar scan rate appears to be 600 scans per min.
% 
% % Loop through each lidar point cloud
% LidFrameNumber = 1:LidarFrameDiv:TotalLidarFrames;
% for lidar_iter = 1:numel(LidFrameNumber)
% 
%     if debugFlag==1 % Display text if Debugging Mode is enabled
%         disp(' > Extracting lidar point cloud data ...')
%     end
% 
%     % Extract the lidar point cloud for that epoch.
%     pcframe = readFrame(pointcloud,LidFrameNumber(lidar_iter));
%     vecr_PCfromL0_xyzL = read_point_cloud(pcframe, SurfCond);
%     % Rotate the point cloud from lidar origin from lidar coords to boom
%     % coordinates.
%     vecr_PCfromL0_tbu = rot_tbu_from_xyzL(vecr_PCfromL0_xyzL, psi(1));
%     % Translate the point cloud from lidar origin to reflected antenna
%     % origin, in boom coordinates.
%     vecr_PCfromL0_tbu = vecr_PCfromL0_tbu + vecr_L0fromR0_tbu;
%     % Rotate from boom coords to enu coords with reflected antenna origin.
%     vecr_PCfromR0_enu = rot_enu_from_tbu(vecr_PCfromL0_tbu, BoomHead);
%     % Crop the point cloud to keep points more than 2 m below R0
%     % and within 10 m north and south of R0.
%     croprows = find(vecr_PCfromR0_enu(:,3)< -2 & ...
%         vecr_PCfromR0_enu(:,1) > 0 & vecr_PCfromR0_enu(:,1) < 10 & ...
%         abs(vecr_PCfromR0_enu(:,2)) < 10);
%     vecr_PCfromR0_cropped_enu{lidar_iter} = vecr_PCfromR0_enu(croprows,:);
%     h1(lidar_iter,1) = -mean(vecr_PCfromR0_cropped_enu{lidar_iter}(:,3));
% end
% % return
% ------Combined lidar and camera processing-------
% Load camera video files
% Add files names of saved camera videos from GNSSR data campaigns
% videoSource{1} = VideoReader('camer1.mp4');%'DataCamp7.mpg'); % Camera 1 Video file
videoSource{2} = VideoReader('camer2.mp4'); % Camera 2 Video file
videoSource{3} = VideoReader('camer3.mp4'); % Camera 3 Video file


% Load camera parameters obtained from Camera Calibration App
% Access my MS thesis for more information or visit website:
% https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html
load('cameraParams.mat');
load('cameraParameters2.mat');
% Without the Computer Vision Toolbox, I needed Shahrukh to convert the
% object to a struct for subsequent use.
% load('cameraParameters_toStruct.mat');
% cameraParams2 = cameraParams2_toStruct;
% cameraParams_A = cameraParams_A_toStruct;
% cameraParams_B = cameraParams_B_toStruct;
% clear cameraParams2_toStruct cameraParams_B_toStruct cameraParams_A_toStruct

% Which Camera video to process?
% 1 = Camera 1
% 2 = Camera 2
% 3 = Camera 3
% CamUse = 3;

% Is Camera 1 video inverted / upside down? (eg. in Test 5)
% 1 = Yes
% 0 = No
InvertCam1 = 0;

% Camera 1 settings
cameraParams{1} = cameraParams2;
K{1} = cameraParams2.IntrinsicMatrix'; % Camera Calibration Matrix 'K' for Cam 1
camhor(1) = 65.985; % Camera Horizontal FOV % Already calculated in our case
camver(1) = 38.815; % Camera Vertical FOV % Already calculated in our case
Cam_el(1) = -45; % Elevation angle % Measured during data campaign
Cam_az(1) = 0; % Azimuth angle % Measured during data campaign
Cam_cr(1) = 3.2; % Misalignment angle about yc axis % Measured in the lab
camOrigin(1,:) = [0 0.012 0.24]; % Lidar origin w.r.t. Camera 1 in Lidar coord in meters % Measured using CAD
ImgSize(1,:) = [1440 2560]; % Image resolution

% Camera 2 settings
cameraParams{2} = cameraParams_B;
K{2} = cameraParams_B.IntrinsicMatrix'; % Camera Calibration Matrix 'K' for Cam 2
camhor(2) = 78.76; % Camera Horizontal FOV % Already calculated in our case
camver(2) = 61.8; % Camera Vertical FOV % Already calculated in our case
Cam_el(2) = -36;%-39; % Elevation angle % Measured during data campaign
Cam_az(2) = -25;%-24; % Azimuth angle % Measured during data campaign
Cam_cr(2) = 0; % Misalignment angle about yc axis % Measured in the lab
camOrigin(2,:) = vecr_L0fromC02_xyzL;%[0.136 -0.135 0.24]; %Lidar origin w.r.t. Camera 2 in Lidar coord % Measured using CAD
ImgSize(2,:) = [1944 2592]; % Image resolution

% Camera 3 settings
cameraParams{3} = cameraParams_A;
K{3} = cameraParams_A.IntrinsicMatrix'; % Camera Calibration Matrix 'K' for Cam 3
camhor(3) = camhor(2);  % Camera Horizontal FOV % Already calculated in our case
camver(3) = camver(2);  % Camera Vertical FOV % Already calculated in our case
Cam_el(3) = -36; %-41; % Elevation angle % Measured during data campaign
Cam_az(3) = 25; %28; % Azimuth angle % Measured during data campaign
Cam_cr(3) = 0; % Misalignment angle about yc axis % Measured in the lab
camOrigin(3,:) = vecr_L0fromC03_xyzL;%[-0.136 -0.135 0.24]; %Lidar origin w.r.t. Camera 3 in Lidar coord % Measured using CAD
ImgSize(3,:) = [1944 2592]; % Image resolution



% Perform backward projection for every X number of lidar frames

% switch CamUse
%     case 1
        % If working on Camera 1
        % Note camera 1 sometimes has missing data, e.g., 15 minutes of
        % data for 20 minutes of data collection duration.
%         TotalVidFrames(1) = floor(videoSource{1}.FrameRate * videoSource{1}.Duration); % Calculate total video frames
%         VidFrameDiv(1) = TotalVidFrames(1)/DivideFrames; % Calculate no. of video frames to skip after each iteration
        
%     case 2
        % If working on Camera 2
        % Camera appears to collect 15 fps.
        TotalVidFrames(2) = videoSource{2}.FrameRate * videoSource{2}.Duration; % Calculate total video frames
        VidFrameDiv(2) = floor(TotalVidFrames(2)/DivideFrames); % Calculate no. of video frames to skip after each iteration
        
%     case 3
        % If working on Camera 3
        TotalVidFrames(3) = videoSource{3}.FrameRate * videoSource{3}.Duration; % Calculate total video frames
        VidFrameDiv(3) = floor(TotalVidFrames(3)/DivideFrames); % Calculate no. of video frames to skip after each iteration
% end

if debugFlag==1 % Display text if Debugging Mode is enabled
    disp(' > Begin processing in while loop ...')
end
    if debugFlag==1 % Display text if Debugging Mode is enabled
        disp(' > Generating figure save paths ...')
    end
    
    % Generate path to save figures
    figsavPath = [pathname filesep 'Figures']; % path to savefigures
    if ~exist(figsavPath,'dir') mkdir(figsavPath); end % Create the save directory if it doesnt exist

% iter = 1;
% % Loop through each lidar point cloud
% for LidFrameNumber = 1:LidarFrameDiv:TotalLidarFrames
% 
%     if debugFlag==1 % Display text if Debugging Mode is enabled
%         disp(' > Obtaining pointcloud ...')
%     end
%     % Extract the lidar point cloud for that epoch.
%     pcframe = readFrame(pointcloud,LidFrameNumber);

    % Loop through each camera image corresponding to that point cloud
    for CamUse = 2:3
%         close all % Close all open figures
        % Generate name for current iteration to use when saving figures:
        CurrName = [TestName{1} ' ' PartName{partnum} ' Cam' num2str(CamUse) ...
            datestr(t_datenum(t_idx),'yymmdd_HHMMUT')];

        % Select which video frame corresponds to the lidar epoch.
        VidFrameNumber = 1 + VidFrameDiv(CamUse)*(t_idx-1);

        % Display Current lidar frame and Current Video Frame
        disp( ['[   Iteration: ' num2str(t_idx) ' / ' num2str(numel(t_datenum)) ...
            '   ][   Epoch: ' datestr(t_datenum(t_idx),'mmm dd, yyyy HH:MM UT') ... 
            '   ][   Camera ' num2str(CamUse) ' Frame: ' ...
            num2str(VidFrameNumber) ' / ' num2str(TotalVidFrames(CamUse)) '   ]']);
%             '   ][   Lidar Frame: ' num2str(LidFrameNumber) ' / ' num2str(TotalLidarFrames) ...

        if CamUse == 1
            InvertCam = InvertCam1;
        else
            InvertCam = 0;
        end
        %eval(['videoSource = videoSource' num2str(CamUse) ';'])
        %     eval(['TotalVidFrames = TotalVidFrames' num2str(CamUse) ';'])
        %eval(['K = K{' num2str(CamUse) '};'])

        img = readindex(videoSource{CamUse},VidFrameNumber); % obtain current image frame from video

        [Results_enu{t_idx,CamUse}, ...
            Results_rgb{t_idx,CamUse}] = ...
            Reconstruct(TestName, CamUse, debugFlag, saveFigs, vecr_PCfromL0_xyzL{t_idx}, ...
            img, ImgSize(CamUse,:), InvertCam,K{CamUse}, cameraParams{CamUse}, ...
            camOrigin(CamUse,:),Cam_az(CamUse),Cam_el(CamUse), ...
            camhor(CamUse),camver(CamUse), ...
            psi(1),BoomHead,Cam_cr(CamUse),SurfCond,figsavPath, ...
            CurrName,t_idx);%, vecr_FZfromR0_enu);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %    Save Results
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     if debugFlag==1 % Display text if Debugging Mode is enabled
%         disp(' > Saving results...')
%     end
%     
%     switch SaveState
%         case 1
%             % If saving results to singular MAT file 
% 
%             % Add every iteration results to new row of variable
%             Results_Lidframe = [Results_Lidframe;{LidFrameNumber}]; % Current Lidar frame no.
%             Results_LidTot = [Results_LidTot;{TotalLidarFrames}]; % Total Lidar frames no.
%             Results_Vidframe = [Results_Vidframe;{VidFrameNumber1}]; % Current Video frame no.
%             Results_VidTot = [Results_VidTot;{TotalVidFrames1}]; % Total Video frames no.
%             Results_enu = [Results_enu;{enu(1:3,:)'}]; % Pixel ENU coordinates
%             Results_rgb = [Results_rgb;{enu(4:6,:)'}]; % 
% 
%         case 2
%             % If saving results in multiple text files
%             
%             ENUresults = table(repmat(CamUse, numel(Results_enu{1}(:,1)),1), ...
%                 Results_enu{1}(:,1),Results_enu{1}(:,2), ...
%                 Results_enu{1}(:,3),Results_rgb{1}(:,1),Results_rgb{1}(:,2), ...
%                 Results_rgb{1}(:,3)); % Generate table with results
%             ENUresults.Properties.VariableNames = {'Camera','Coord E','Coord N','Coord U','Pixel Red','Pixel Green','Pixel Blue'}; % Add column labels
%             writetable(ENUresults,[pathname filesep  TestName{1} '_' PartName{partnum} '_Cam' num2str(CamUse) '_LidarFrame' num2str(LidFrameNumber) '.txt']); % save file to results Path
% 
%     end

    end % for CamUse

    % For each lidar epoch, find all the ENU points from both cameras that
    % are within a given PRN's Fresnel zone (FZ).  Take the mean red value
    % over all those pixels.
    % This is where I should fold in Roohollah's code that does that
    % calculation.
    
    % Translate all pixel ENU points to origin at reflected antenna.
%     vec_PCfromR0_enu = Results_enu{1} - 

%     if debugFlag==1 % Display text if Debugging Mode is enabled
%         disp(' > Preparing for next iteration...')
%     end
%     iter = iter+1;

% end % for LidFrameNumber

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Finalize results                                                                            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if debugFlag==1 % Display text if Debugging Mode is enabled
        disp(' > Finalize saving results...')
end

switch SaveState
    case 1
        % If saving results to singular MAT file 
        
        % Generate Table by combining each results variable as a column
        ImageResults = table(Results_Lidframe, Results_LidTot, Results_Vidframe, Results_VidTot, Results_enu, Results_rgb);
        
        %Add column labels to results table
        ImageResults.Properties.VariableNames = {'Lidar Frame no.','Total Lidar Frames','Video Frame no.','Total Video Frames','Image in Lidar Coords','Pixel RGB'};
       
        % save results to results path as MAT file
        matfile = fullfile(pathname, filesep, [TestName{1} '_' PartName{partnum} '_Cam' num2str(CamUse) '.mat']); % Results MAT file name
        save(matfile,'ImageResults'); % save MAT file to results Path
disp(['Results saved to: ' pathname]);
        
    case 2
        % If saving results in multiple text files
        
end

% Display completion text
disp('----------------------------------------');
disp('PROCESSING COMPLETED!');

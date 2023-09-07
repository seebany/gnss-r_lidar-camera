
% %==========================================================================
% %--------SP-LiDAR almanac info based
% %--------Roohi 6/15/2020
% %==========================================================================
% % this script creates an almanac based SP-LiDAR-Camera  Map for whole 20 minute
% % Roohi UPDATE: 4/20/2021 :
% %  In this funciton we find the neighbor pixels brightness value : to compare
% %  it with the SR for given SP
%
% %==========================================================================
% %-----UPDATE : Rootendhi 4/23/2021 :
% % SR and pixel brighness :
% %--------------------------------------------------------------------------
% % UPDATE ROOHI : 4/24/2021 :
% % this is a special case for the brightnes only not image and PHI and other
% % variable :::
%
%
% % I do not use LiDAR and Camera and phi and other variaables OK
% % roohi chi man mikham : this for seccond paper not a genreal Code :
%
%
% %==========================================================================
% % update: Roohi : 5/17/2021 :
% % combine all 3 cameras" ok :
% % rotation of fernal zone:
% % pick pixels inside the rotated fernal zone:
% % find the mean value of Red of those pixels
% % correlation with SR:
% %==========================================================================
%
% % UPDATE:::: 5/30/2021 :
% % the for comoputeing the FZ equatin form Larson is used:
% % From BOOK : Hristov HD (2000) Fresnel zones in wireless links, zone plate lenses
% % and antennas. Artech House.
% %
% % Seebany Datta-Barua
% % 11 Dec 2022
% % Merging with lidar and camera processing codes, and updating
% organization to follow that described in hardware manuscript submitted to NAVI.

%==========================================================================
%------------clear workspace ---------------------------------------------

clc
clear variables
close all

%--------------------------------------------------------------------------
% Initialize data campaign settings.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Load Initial Files/Values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path to read GNSSR Camera Lidar data
% addpath(genpath('E:\GNSSR data'));
% addpath(genpath('C:\Users\shark\Documents\MATLAB\GNSSR\Test Data'));
addpath(genpath('/Users/seebany/Dropbox/MATLAB/mfiles/gnss-r/lidar_camera/'))
addpath(genpath('/Users/seebany/Dropbox/MATLAB/mfiles/navutils/'))
addpath(genpath('/Users/seebany/Dropbox/MATLAB/mfiles/time_conversion/'))

% Add Path to save results
% pathname = 'C:\Users\shark\Desktop\New folder';
pathname = '/Users/seebany/Dropbox/MATLAB/outfiles/gnss-r/';
if ~exist(pathname,'dir')
    mkdir(pathname);
end % Create the data save directory if it doesn't exist

% 
figsavPath = [pathname 'Figures' filesep];
if ~exist(figsavPath,'dir')
    mkdir(figsavPath);
end % Create the figure save directory if it doesn't exist

% Add Test name and Part no. to be used when saving results
% eg. 'Test7' 'Part1'
% TestName{1} = 'Test7'; % Data campaign test number
% TestName{2} = 'Part1'; % Part of that test
TestName{1} = 'Test11'; % Data campaign test number
startpart = 1;
endpart = 6;

% Generate and save all Figures?
% This is to allow figures to be generated and saved to the results path.
% 1 = Yes
% 0 = No
saveFigs = 0;

% Save results as
% 1 = singular MAT file (very memory intensive while processing)
% 2 = multiple text files
SaveState = 0;%2;

% Debug the code?
% This will cause Matlab to pause at the end of each iteration of cam2enu and plot the figures.
% This alone will not cause figures to save. They will only save if saveFigs = 1.
% 1 = Yes
% 0 = No
debugFlag = 0;

% List all the measured relative position vectors, in transverse, boom, up
% coordinates.
vecr_D0fromR0_tbu = [0, 0, 0.57];
vecr_R0fromL0_tbu = [0, 0.43, 0.40];
vecr_L0fromR0_tbu = -vecr_R0fromL0_tbu;
% List relative position vectors in lidar coordinate system xL, yL, zL.
vecr_L0fromC01_xyzL = [0 -0.135, -.24]; % Guessing based on symmetry.
vecr_L0fromC02_xyzL = [0.136, -0.135, 0.24];
vecr_L0fromC03_xyzL = [-0.136, -0.135, 0.24];

% List all orientation angles not defined elsewhere.
kappa = -45; % deg, angle between reflected antenna axis of symmetry and 
% boom direction.

%--------------------Select PRNs if known.
prnlist = [8, 16, 26, 27];
%--------------------Select times for calculation.
duration = 20/60/24; % Each test part is 20 minutes long.
time_step_min = 1; % Minutes
time_step_sec = time_step_min*60; % Seconds

%----------------------Test date-------------------------------------------
if debugFlag==1 % Display text if Debugging Mode is enabled
    disp('----------------------------------------')
    disp('DEBUGGING MODE ENABLED!')
    disp('----------------------------------------')
    disp(' > All figures will be plotted.')
    disp(' > Progess through the code can be tracked in the command window.')
    disp(' > Execution paused to allow variables to be examined')
    disp(' > Execution will pause again at the end of the cam2enu function to allow for figures and variables to be examined.')
    disp('----------------------------------------')
    disp('PROCESSING PAUSED')
    s = dbstack; % Obtain current line number
    lineNum = s.line + 3; % Identify line number a 3 lines ahead of the last line
    eval(['dbstop at ' num2str(lineNum)]) % Set breakpoint on next line to pause execution
    eval(['dbclear at ' num2str(lineNum)]) % Once execution is resumed, remove breakpoint
    disp('RESUMING PROCESSING')
    disp('----------------------------------------');
end

% Initialize list of part names.
for partnum = startpart:endpart
    PartName{partnum} = ['Part' num2str(partnum)]; % Part of that test
end

% Read in the number of parts based on the directory path.
for partnum = startpart:endpart
    tic

    % Add the read directory folder.
    addpath(['/Users/seebany/Dropbox/Data/gnss-r/lidar_camera/LM_' ...
        TestName{1} filesep PartName{partnum}]);
    % Generate name for current iteration to use when saving figures:
    CurrName = [TestName{1} '_' PartName{1}];
    if ~exist([pathname, CurrName],'dir')
        mkdir([pathname, CurrName]);
    end % Create the save directory if it doesnt exist

    % Read from the lidar text file the start time for the part.
    fid = fopen('LiDAR.txt');
    % Skip the first line.
    fgetl(fid);
    % Read the second line.
    line = fgetl(fid);
    charidx = findstr(line, 'DATE');
    % Conversion from local time to universal time, in hours.
    LT2UT = 6; % hrs
    t_0 = datenum(line(charidx + 5:charidx + 5 + 18), ...
        'yyyy-mm-dd HH:MM:SS') + LT2UT/24; % UT start time.
    fclose(fid);

    %--------------------Estimate direct antenna position
    %--------------------This is \vec{r}^{D_0} relative to WGS-84 origin.
    %--------------------Option 1: Hardcode an approximate position
    lat_direct = 41.837998;  % deg N              %41.837977 ;
    lon_direct = -87.606115  ; % deg E
    h_approx = 172; % m reflected antenna height, based on ?
    % direction='east'; % boom azimuth direction
    h_direct = h_approx + vecr_D0fromR0_tbu(3);

    %--------------------Option 2: compute via navigation solution
    % To be filled in.


    %----------------------Estimate reflected antenna position h_antenna-------
    lat_reflect = lat_direct;
    lon_reflect = lon_direct;
    h_antenna = h_direct - vecr_D0fromR0_tbu(3);

    dt = time_step_min/60/24; % Epoch cadence is one minute as a fractional day for datenum, for calculation of MRV.
    t_f = t_0 + duration - dt; % Don't end with the last timestamp being the 21st minute.

    %==========================================================================
    %--------------------Compute satellite positions ---------------------------------------
    %--------------------Option 1: compute via almanac obtained from
    %--------------------https://www.navcen.uscg.gov/archives
    [t_gps, prn_gps, vecr_svfromR0_ned] = satpos_from_alm_for_gnssr(t_0, t_f, time_step_sec, ...
        lat_reflect, lon_reflect, h_antenna);

    % Compute azimuth and elevation, in rad.
    [az, el] = ned2azel(vecr_svfromR0_ned);

    % Note: az_el_table will be nx5 because t_gps = [gps_week gps_sec].
    az_el_table = [t_gps, prn_gps, az, el];
    % SDB 12/22/22 replace the first 2 columns with t_datenum, but keep two
    % time columns for backward compatibility with nx5 size.
    az_el_table(:,1) = reshape(repmat([t_0:dt:t_f], ...
        numel(unique(prn_gps)),1), [],1);
    az_el_table(:,2) = az_el_table(:,1);

    %--------------------Option 2: compute sv az, el via navigation solution
    % To be filled in.

    %---------------------------------------------------------------

    % Plot sky plot Figure
    if saveFigs==1 || debugFlag==1 % Check if plotting is requested
        % Select the first time
        plot_time = t_f;
        timerows = find(az_el_table(:,1) == plot_time);
        az_deg = az_el_table(timerows,4)*180/pi';
        el_deg = az_el_table(timerows,5)*180/pi';
        prn = az_el_table(timerows,3);
        cols = find(el_deg > 0);

        Figr(16) = figure(16);
        clf
        h = skyPlot(az_deg(cols), el_deg(cols), prn(cols),' bo');
        title(['Sky Plot of Satellites Above Horizon at ' datestr(plot_time, 'mmm dd, yyyy HH:MM UT')])

        if saveFigs==1 % Check if Saving figures is requested
            figsav1 = [figsavPath filesep num2str(get(gcf,'Number'))]; % path to save this figure, in folder with figure number
            if ~exist(figsav1,'dir')
                mkdir(figsav1);
            end % Create the save directory if it doesn't exist
            saveas(gcf,[figsav1 filesep 'skyplot' datestr(plot_time,'HHMM') '.epsc']) % Save figure as an image
        end
    end

    % -------------------------------------------------------------------------
    % ----------------Use lidar to compute the surface
    % height relative to the reflected antenna, h1.
    % mean_water_ht_from_lidar = estimate_
    t_datenum = t_0:dt:t_f;
    [h1,vecr_PCfromL0_xyzL, BoomHead,psi(1)] = get_surface_depth_from_lidar(t_datenum, ...
        vecr_L0fromR0_tbu, debugFlag);
    % h1 is created in lidar_camera_processing. [nepochs x 1] in m.

    % Compute the antenna zenith angle, phi between the satellite and the
    % reflection antenna zenith.
    phi_sv = compute_phi(vecr_svfromR0_ned, ...
        kappa, BoomHead, prnlist, t_datenum, az_el_table);

    % ---------------Compute the specular point positions based on h1, az, el.
    % Based on the first skyplot, these are the satellites that should scan
    % over the surface.

    %--------------------------------------------------------------------------
    % Compute the position of the Fresnel zone x0, d = lambda/2 to compute
    % Fresnel zone a, b
    lambda = 3e8/1575.42e6; % wavelength of GPS L1 signal.
    % theta should be the angle defined in Fig 1 of the NAVI 2022 paper, and
    % just from from 0 to 2*pi.
    epsilon = 1e-3; % ensures the tan doesn't go singular and the ellipse is smooth and continuous.
    theta = [-pi/2+epsilon:0.01:pi/2+epsilon]';

    % Initialize a calculation of the SP and FZ parameters.
    x00 = zeros(numel(t_datenum),numel(prnlist));
    a_L = zeros(numel(t_datenum),numel(prnlist));
    b_L = zeros(numel(t_datenum),numel(prnlist));
    area_L = zeros(numel(t_datenum),numel(prnlist));
    vecr_SPfromR0_xyzSP = zeros(numel(t_datenum),numel(prnlist),3);

    % Compute the FZ center and semi-major and semi-minor axes, and area.
    for prn_idx = 1:numel(prnlist)
        % Find the rows of the az_el_table that correspond to the prn sought.
        prnrows = find(az_el_table(:,3) == prnlist(prn_idx));
        el = az_el_table(prnrows,5);
        % eq from Larson paper  :
        half_wavelength=lambda/2;
        x00(:,prn_idx) = h1./tan(el) + ...
            half_wavelength./(sin(el).* tan(el));

        % Try cutting the area in half by scaling a and b by 1/sqrt(2). SDB
        % 1/3/23. Works better than full.  Try cutting area by 1/4th next.
        % And then area by 100th.
        b_L(:,prn_idx) = sqrt(2 .* half_wavelength .* h1 ./ sin(el) + ...
            (half_wavelength ./ sin(el)).^2);%./ sqrt(10^2);
        a_L(:,prn_idx) = b_L(:,prn_idx)./sin(el);
        area_L(:,prn_idx) = pi.*a_L(:,prn_idx).*b_L(:,prn_idx);

        % Compute vector from R0 to SP in SP-aligned coordinates.
        vecr_SPfromR0_xyzSP(:,prn_idx, 1) = x00(:,prn_idx); % m
        vecr_SPfromR0_xyzSP(:,prn_idx, 3) = -h1; % m
    end
%     % Save geometry data computed so far.
%     save([pathname 'FZ_area' ...
%         TestName{1} PartName{partnum} '.mat'], 't_datenum', 'area_L', 'phi_sv'); %,'pixelArea','meanRed');%); %
% 
% %--------------Compute the surface reflectivity.--------------------------
%     SR = compute_sr_from_snr(TestName{1}, PartName{partnum}, vecr_SPfromR0_xyzSP,vecr_svfromR0_ned, ...
%         kappa, BoomHead, prnlist, t_datenum, az_el_table);
%     % Save SR data computed so far.
%     save([pathname 'sr' TestName{1} PartName{partnum} '.mat'], 'SR', 't_datenum');
%     toc
% end


    % For each time and each prn, create a set of points that outline the FZ
    % ellipse.
    for t_idx = numel(t_datenum):numel(t_datenum)
        % Use lidar_camera_processing.m to get the pixel point
        % cloud (PPC) vectors and associated RGB values.
        lidar_camera_processing;

    end

    % Rotate the relative position vector from C02 to R0 into enu
    % coords.
    vecr_R0fromC02_enu = rot_enu_from_tbu(vecr_R0fromL0_tbu,BoomHead) + ...
        rot_enu_from_tbu(...
        rot_tbu_from_xyzL(vecr_L0fromC02_xyzL, psi(1)), ...
        BoomHead);
    vecr_C02fromR0_enu = - vecr_R0fromC02_enu;

    % Same procedure to get R0 relative to C03 in enu.
    vecr_R0fromC03_enu = rot_enu_from_tbu( ...
        vecr_R0fromL0_tbu + ...
        rot_tbu_from_xyzL(vecr_L0fromC03_xyzL, psi(1)), ...
        BoomHead);
    vecr_C03fromR0_enu = - vecr_R0fromC03_enu;

    % For each time and each prn, create a set of points that outline the FZ
    % ellipse. Then compute the mean red values.
    meanRed = nan*zeros(numel(t_datenum), numel(prnlist));
    pixelArea = nan*zeros(numel(t_datenum), numel(prnlist));
    for t_idx = numel(t_datenum):numel(t_datenum)
        datevec(t_datenum(t_idx))
        tic
        % Add position vectors to get position of FZ from C02 in
        % enu coords. Matlab R2022b appears to let you add the nx3
        % vector to a 1x3 vector, by adding the 1x3 vector to all n
        % rows.
        %                 vecr_FZfromC02_enu = vecr_FZfromR0_enu + vecr_R0fromC02_enu;
        vecr_PPC2fromR0_enu = Results_enu{t_idx,2} + vecr_C02fromR0_enu;
        %                 vecr_FZfromC03_enu = vecr_FZfromR0_enu + vecr_R0fromC03_enu;
        vecr_PPC3fromR0_enu = Results_enu{t_idx,3} + vecr_C03fromR0_enu;

        % Find the overlap between the two camera images.
        % This appears to make little difference in the MRVs.
        finrows = find(isfinite(vecr_PPC2fromR0_enu(:,1)) & ...
            isfinite(vecr_PPC2fromR0_enu(:,2)));
        delaunay3in2 = delaunayTriangulation( ...
            vecr_PPC2fromR0_enu(finrows,1),vecr_PPC2fromR0_enu(finrows,2));
        outline = convexHull(delaunay3in2);
        % Find the points in camera 3 image that fall within the convex
        % hull of the camera 2 image points.
        overlap3in2 = inpolygon(vecr_PPC3fromR0_enu(:,1), ...
            vecr_PPC3fromR0_enu(:,2), delaunay3in2.Points(outline,1), ...
            delaunay3in2.Points(outline,2));

        % Repeat for finding overlap of 2 in 3.
        clear outline finrows
        finrows = find(isfinite(vecr_PPC3fromR0_enu(:,1)) & ...
            isfinite(vecr_PPC3fromR0_enu(:,2)));
        delaunay2in3 = delaunayTriangulation( ...
            vecr_PPC3fromR0_enu(finrows,1),vecr_PPC3fromR0_enu(finrows,2));
        outline = convexHull(delaunay2in3);
        % Find the points in camera 3 image that fall within the convex
        % hull of the camera 2 image points.
        overlap2in3 = inpolygon(vecr_PPC2fromR0_enu(:,1), ...
            vecr_PPC2fromR0_enu(:,2), delaunay2in3.Points(outline,1), ...
            delaunay2in3.Points(outline,2));

        % Use the overlapping area to find the mean and stdev of the
        % camera 3 histogram to scale to fit the mean and stdev of the
        % camera 2 histogram.
        [sig2, mu2] = std(Results_rgb{t_idx, 2}(overlap2in3,:),0,1);
        [sig3, mu3] = std(Results_rgb{t_idx, 3}(overlap3in2,:),0,1);
        newHist = (Results_rgb{t_idx, 3} ...
            - repmat(mu3, size(Results_rgb{t_idx,3},1),1)) ...
            ./ repmat(sig3, size(Results_rgb{t_idx,3},1),1) ...
            .* repmat(sig2, size(Results_rgb{t_idx,3},1),1) ...
            + repmat(mu2, size(Results_rgb{t_idx,3},1),1);

        if saveFigs==1 || debugFlag==1 % Check if plotting is requested
            figure
            subplot(311)
            title('Red histogram')
            histogram(Results_rgb{t_idx, 2}(overlap2in3,1));
            hold on
            histogram(Results_rgb{t_idx, 3}(overlap3in2,1));
            histogram(newHist(overlap3in2,1));

            subplot(312)
            title('Green histogram')
            histogram(Results_rgb{t_idx, 2}(overlap2in3,2));
            hold on
            histogram(Results_rgb{t_idx, 3}(overlap3in2,2));
            histogram(newHist(overlap3in2,2))

            subplot(313)
            title('Blue histogram')
            histogram(Results_rgb{t_idx, 2}(overlap2in3,3));
            hold on
            histogram(Results_rgb{t_idx, 3}(overlap3in2,3));
            histogram(newHist(overlap3in2,3))
        end

            % Replace the original camera 3 RGB values with the normalized
            % ones.
            Results_rgb{t_idx,3} = newHist;

            % Turn the overlapping points of camera 3 to nans.
            Results_rgb{t_idx,3}(overlap3in2,:) = nan;
            Results_enu{t_idx,3}(overlap3in2,:) = nan;
            vecr_PPC3fromR0_enu(overlap3in2,:) = nan;

        % Plot Figure
        if 1 %t_idx == 1%saveFigs==1 || debugFlag==1 % Check if plotting is requested
            %             % Select the first time
            %             timerows = find(datenum(gps2utc(az_el_table(:,1:2))) == t_0);
            % Plot Fresnel zone relative to R0 in East-North-Up coordinates
            Figr(18) = figure(18);
            %         clf
            xlabel('East [m]', 'FontSize',14)
            ylabel('North [m]', 'FontSize',14)
            title(['SPs with FZ relative to R0 on Lake \newline Michigan on ' ...
                datestr(t_datenum(t_idx), 'mmm dd, yyyy HH:MM UT')], 'FontSize',14)
            axis equal
            axis([-1 5 -5 5])
            grid on
            hold on
            % Plot the camera 2 pixels relative to R0.
            scatter(vecr_PPC2fromR0_enu(:,1), ...
                vecr_PPC2fromR0_enu(:,2), ...
                [], Results_rgb{t_idx,2}, 'filled');%, '.');
            % Plot the camera 3 pixels relative to R0.
            scatter(vecr_PPC3fromR0_enu(:,1), ...
                vecr_PPC3fromR0_enu(:,2), ...
                [], Results_rgb{t_idx,3}, 'filled');%, '.');
          end % if t_idx == 1

        for prn_idx = 1:numel(prnlist)
            % Compute the points of the perimeter of each Fresnel zone.
            % x^2/a^2 + y^2/b^2 = 1, solve for x with y = x*tan(theta).
            % These are in the SP coordinate system. Upper half, lower
            % half of the ellipse by using +/- for the sqrt.
            vecr_FZfromSP_xyzS(:,1) = [-a_L(t_idx,prn_idx).*b_L(t_idx,prn_idx) ./ ...
                sqrt(b_L(t_idx,prn_idx).^2 + a_L(t_idx,prn_idx).^2.*(tan(theta).^2));
                a_L(t_idx,prn_idx).*b_L(t_idx,prn_idx) ./ ...
                sqrt(b_L(t_idx,prn_idx).^2 + a_L(t_idx,prn_idx).^2.*(tan(theta).^2))];
            % The ordering of theta helps the ellipse be continuous and convex
            % rather than bisected at the semi-minor axis.
            vecr_FZfromSP_xyzS(:,2) = vecr_FZfromSP_xyzS(:,1).*[tan(theta); -tan(pi-theta)];
            vecr_FZfromSP_xyzS(:,3) = zeros(size(vecr_FZfromSP_xyzS(:,1)));

            % Find the rows of the az_el_table that correspond to the prn sought.
            %         prnrows = find(az_el_table(:,3) == prnlist(prn_idx));
            %         el = az_el_table(prnrows,5);

            % Rotate them to the ENU coordinate system
            % The rotation matrix from SP major axis coordinates to ENU.
            az_idx = find(az_el_table(:,3) == prnlist(prn_idx));% & az_el_table

            % Assumes they're listed
            % in chronological order in az_el_table.
            az = az_el_table(az_idx(t_idx),4);
            % Create the rotation matrix.
            enuRxyzS = [sin(az), -cos(az), 0;
                cos(az), sin(az), 0;
                0,   0,   1];

            % Rotate the FZ and the SP position vectors to ENU coordinates.
            vecr_FZfromSP_enu = quicktimes(enuRxyzS, vecr_FZfromSP_xyzS')';
            vecr_SPfromR0_enu = quicktimes(enuRxyzS,squeeze(vecr_SPfromR0_xyzSP(t_idx,prn_idx,:)))';
            % Translate the ellipse perimeter to be relative to the
            % reflected antenna.
            vecr_FZfromR0_enu = vecr_FZfromSP_enu + ...
                repmat(vecr_SPfromR0_enu, size(vecr_FZfromSP_enu,1),1);

            % Use inpolygon to identify which pixels are within the
            % ellipse.
            IN{2} = inpolygon(vecr_PPC2fromR0_enu(:,1), vecr_PPC2fromR0_enu(:,2), ...
                vecr_FZfromR0_enu(:,1), vecr_FZfromR0_enu(:,2));

            % Then use inpolygon to identify which pixels are within the
            % ellipse.
            IN{3} = inpolygon(vecr_PPC3fromR0_enu(:,1), vecr_PPC3fromR0_enu(:,2), ...
                vecr_FZfromR0_enu(:,1), vecr_FZfromR0_enu(:,2));

            % Compute the area of the pixels that contribute to the MRV
            % calculation.
            if any(IN{2}) || any(IN{3})
                %         finrows = find(isfinite(vecr_PPC2fromR0_enu(:,1)) & ...
                %             isfinite(vecr_PPC2fromR0_enu(:,2)));
                clear delaunay outline finrows2 finrows3
                % Update the calculation because any nans will make the
                % total area nan.
                PPCinEast = [];
                PPCinNorth = [];
                if any(IN{2}) 
                    finrows2 = find(IN{2});
                    PPCinEast = [PPCinEast; 
                        vecr_PPC2fromR0_enu(finrows2,1)];
                    PPCinNorth = [PPCinNorth; 
                        vecr_PPC2fromR0_enu(finrows2,2)];
                end
                if any(IN{3})
                    finrows3 = find(IN{3});
                    PPCinEast = [PPCinEast; 
                        vecr_PPC3fromR0_enu(finrows3,1)];
                    PPCinNorth = [PPCinNorth; 
                        vecr_PPC3fromR0_enu(finrows3,2)];
                end
                
                delaunay = delaunayTriangulation(PPCinEast, PPCinNorth);
                outline = convexHull(delaunay);
                pixelArea(t_idx,prn_idx) = polyarea(delaunay.Points(outline,1), ...
                    delaunay.Points(outline,2));
            end % if ~isempty(IN{2}) & ~isempty(IN{3})

            if 1 %t_idx == 1
                % Return to the already opened figure.
                figure(18)
                % Plot the Fresnel zone partly transparent.
                h = patch(vecr_FZfromR0_enu(:,1), vecr_FZfromR0_enu(:,2),'r');
                set(h,'FaceColor', 'red', 'FaceAlpha', 0.1)

                % Plot and label the SP
                scatter(vecr_SPfromR0_enu(:,1),vecr_SPfromR0_enu(:,2), ...
                    [], [1 0.5 0],'filled','o');
                text(vecr_SPfromR0_enu(:,1),vecr_SPfromR0_enu(:,2), ...
                    ['PRN-',num2str(prnlist(prn_idx))], ...
                    'horizontal','left', 'vertical','bottom','FontSize', 12);
                % Plot R0 origin
                scatter(0,0,200,'gX','LineWidth', 4);

                if 1 %saveFigs==1 % Check if Saving figures is requested
                    figsav1 = [figsavPath filesep num2str(get(gcf,'Number'))]; % path to save this figure, in folder with figure number
                    if ~exist(figsav1,'dir')
                        mkdir(figsav1);
                    end % Create the save directory if it doesnt exist
                    orient portrait
                    saveas(gcf,[figsav1 filesep 'Fig' num2str(get(gcf,'Number')) ' ' CurrName '36deg.jpg']) % Save figure as an image
                end

            end % if t_idx == 1
            % Compute the mean red value for that time and prn, over all
            % cameras' images.
            RedCam2 = Results_rgb{t_idx, 2}(:,1);
            RedCam3 = Results_rgb{t_idx, 3}(:,1);
            meanRed(t_idx, prn_idx) = mean([RedCam2(IN{2}); RedCam3(IN{3})]);%...
%                 ./pixelArea(t_idx, prn_idx);
            if debugFlag==1 % Display text if Debugging Mode is enabled
                disp(' > Preparing for next prn iteration...')
            end
        end % for prn_idx
        if debugFlag==1 % Display text if Debugging Mode is enabled
            disp(' > Exiting prn iteration loop...')
        end
        toc
    end % for t_idx
        % Save data computed so far.
        save([pathname 'normed_mrv_and_FZpixel_area_excluding_redundant_pixels' ...
            TestName{1} PartName{partnum} '.mat'], 't_datenum', 'pixelArea','meanRed');%); %'area_L'); , 

    if debugFlag==1 % Display text if Debugging Mode is enabled
        disp(' > Exiting time iteration loop...')
    end
    rmpath(['/Users/seebany/Dropbox/Data/gnss-r/lidar_camera/LM_' ...
        TestName{1} filesep PartName{partnum}]);

    toc
end % for partnum

plot_sr_mrv_results(TestName, PartName, pathname, prnlist);
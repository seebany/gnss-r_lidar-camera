%=============== Surface reflectivity ==================================

% For 20 minute : 3/8/2021
% roohi surface reflectivity
% input :
% Based on  : based on  Chew_Nature 2018 paper
% equation :
%SR ? SNR ? 10 logPr ? 10 logG ? 10 logG ? 20 log? + 20 log(Rsr + Rts ) + 20 log(4?)
% what I need : Pr , Gr , Gt, Pr, Rt and Rr , SNR
%
% Created by Seebany Datta-Barua based on NAVI paper description and
% Roohollah Parvizi's code.
% 22 Dec 2022

% updata SR for the almanc info
% read SNR first/computed it
% compute SP first

function SR = compute_sr_from_snr(TestName, PartName, vecr_SPfromR0_xyzSP,vecr_svfromR0_ned, ...
    kappa, BoomHead, prnlist, t_datenum, az_el_table)

%  1: visiable SV = el , az X, Y , Z
% 2: el,az == antnna gain Gr,
% 3: SP , rsr, rst
% 4: SNR

% ========================================================================
% Initialize
% Rsr = zeros(numel(t_datenum), numel(prnlist));
Pt=17.40; %                  in dbW :         55 Watt From GPS BOOK
lambda=0.19;  % m, GPS L1 wavelength
vecr_SPfromR0_enu = nan(size(az_el_table, 1),3);

%--------------Compute Rsr for all times and all PRNs in the list.--------/
Rsr = sqrt(sum(vecr_SPfromR0_xyzSP.^2,3)); %ntimes x nprns

%--------------Compute Rts----------------------------------------------/
% Reorder the vector to be ENU coordinates.
vecr_svfromR0_enu = [vecr_svfromR0_ned(:,2), vecr_svfromR0_ned(:,1), ...
    - vecr_svfromR0_ned(:,3)];

% Loop over time and prns.
for t_idx = 1:numel(t_datenum)
    for prn_idx = 1:numel(prnlist)

            rows = find(az_el_table(:,1) == t_datenum(t_idx) & ...
                az_el_table(:,3) == prnlist(prn_idx));
            if isempty(rows) keyboard; end
            % Rotate them to the ENU coordinate system
            % The rotation matrix from SP major axis coordinates to ENU.
%             az_idx = find(az_el_table(:,3) == prnlist(prn_idx));% & az_el_table

%             % Assumes they're listed
%             % in chronological order in az_el_table.
            az = az_el_table(rows,4);
            % Create the rotation matrix.
            enuRxyzS = [sin(az), -cos(az), 0;
                cos(az), sin(az), 0;
                0,   0,   1];

            % Rotate the SP position vectors to ENU coordinates.
            vecr_SPfromR0_enu(rows,:) = quicktimes(enuRxyzS,squeeze(vecr_SPfromR0_xyzSP(t_idx,prn_idx,:)))';

            % Compute relative position vector.
            vecr_svfromSP_enu = vecr_svfromR0_enu(rows,:) - vecr_SPfromR0_enu(rows,:);

            % Compute the distance in m.
            Rts(t_idx, prn_idx) = sqrt(sum(vecr_svfromSP_enu.^2,2));
    end
end

%--------------Compute Gr ----------------------------------------------/
% Compute angle phi between the reflection antenna's
%             % \hat{y}_r and vecr_SPfromR0.
vecr_SPfromR0_ned = [vecr_SPfromR0_enu(:,2), vecr_SPfromR0_enu(:,1), ...
    -vecr_SPfromR0_enu(:,3)];
phi = compute_phi(vecr_SPfromR0_ned, ...
        kappa, BoomHead, prnlist, t_datenum, az_el_table); % ntimes x nprns

baseline_gain = 42*ones(size(phi)); % dB, baseline gain of antenna
dBloss = nan*zeros(size(phi)); % dB lost, charted according to antenna manufacturer

% Store as a table and interpolate between the values.
loss_table(:,1) = [0; 20; 30; 40; 55; 65; 70; 75; 85; 90; 105; 120; 140; 150; 180];
loss_table(:,2) = [0; -3; -4; -5; -6; -7; -8; -9; -10; -11; -14; -20; -22; -30; -40];

% Linearly interpolate the loss amounts for each phi angle.
dBloss = interp1(loss_table(:,1), loss_table(:,2), phi);

% % Antenna gain table manually determined by inspection of manufacturer's antenna gain map.
% dBloss(abs(phi) >= 0 & abs(phi) < 20) = -3; % dB
% dBloss(abs(phi) >= 20 & abs(phi) < 30) = -4; % dB
% dBloss(abs(phi) >= 30 & abs(phi) < 40) = -5; % dB
% dBloss(abs(phi) >= 40 & abs(phi) < 55) = -6; % dB
% dBloss(abs(phi) >= 55 & abs(phi) < 65) = -7; % dB
% dBloss(abs(phi) >= 65 & abs(phi) < 70) = -8; % dB
% dBloss(abs(phi) >= 70 & abs(phi) < 75) = -9; % dB
% dBloss(abs(phi) >= 75 & abs(phi) < 85) = -10; % dB
% dBloss(abs(phi) >= 85 & abs(phi) < 90) = -11; % dB
% dBloss(abs(phi) >= 90 & abs(phi) < 105) = -14; % dB
% dBloss(abs(phi) >= 105 & abs(phi) < 120) = -20; % dB
% dBloss(abs(phi) >= 120 & abs(phi) < 140) = -22; % dB
% dBloss(abs(phi) >= 140 & abs(phi) < 150) = -30; % dB
% dBloss(abs(phi) >= 150 & abs(phi) < 180) = -40; % dB

Gr = baseline_gain + dBloss; % dB

% --------------Load the SNR file Roohollah computed on the server.------/
snrpath = ['/Users/seebany/Dropbox/Data/gnss-r/snr/' ...
    'LM_' TestName filesep PartName filesep];
load([snrpath,'SNR2_PRNs_20_Min', '.mat'])
% 
% SNR2_PRNs_Min is a cell array of nprns x ntimes e.g., 32x20.
SNR=(SNR2_PRNs_Min(prnlist,:))';
% Each cell appears to contain 1x60 elements. I guess this is 1 Hz SNR.
% Compute mean over each minute for each prn.
for t_idx = 1:numel(t_datenum)
    for prn_idx = 1:numel(prnlist)
        meanSNR(t_idx, prn_idx) = mean(SNR{t_idx, prn_idx});
    end
end

% --------------Compute the SR
% SDB thinks since Gr is already in dB and Pt is already in dB, those
% should not be 10log10'ed. 12/22/22.
    SR =    10*log10(meanSNR) ...
          - Pt ... 10*log10(Pt) ...
          - 10*log10(1) ...
          - Gr ... 10*log10(Gr) ...
          - 10*log10(lambda) ...
          + 20*log10(Rsr + Rts) ...
          + 20*log10(4*pi);


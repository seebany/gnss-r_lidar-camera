function h = plot_sr_mrv_results(TestName, PartName, pathname, prnlist)

% Load all the results.
startpart = 1;
endpart = 6;

meanRedAllParts = [];
meanRedppaAllParts = [];
pixelAreaAllParts = [];
FZAreaAllParts = [];
phi_svAllParts = [];
deltaSRAllParts = [];
t_datenumAllParts = [];
t_datenumppaAllParts = [];
sr_datenumAllParts = [];
for partnum = startpart:endpart
    partnum
    % Load surface reflectivity values if they exist.
    try
        load([pathname 'sr' TestName{1} PartName{partnum} '.mat'], ...
            'SR', 't_datenum');
        if verLessThan('matlab', 'R2022b')
            deltaSRAllParts = [deltaSRAllParts; SR - repmat(SR(1,:),size(SR,1),1)];
        else
        deltaSRAllParts = [deltaSRAllParts; SR - SR(1,:)]; % ntimes x nprns
        end
        sr_datenumAllParts = [sr_datenumAllParts, t_datenum]; % 1 x ntimes
    catch
    end
    % Load FZ area values if they exist.
    try
        load([pathname 'FZ_area' TestName{1} PartName{partnum} '.mat'], ...
            'area_L', 'phi_sv');
        FZAreaAllParts = [FZAreaAllParts; area_L]; % ntimes x nprns
        phi_svAllParts = [phi_svAllParts; phi_sv];
%         sr_datenumAllParts = [sr_datenumAllParts, t_datenum]; % 1 x ntimes
    catch
        FZAreaAllParts = [FZAreaAllParts; nan*zeros(numel(t_datenum),numel(prnlist))]; % ntimes x nprns
        phi_svAllParts = [phi_svAllParts; nan*zeros(numel(t_datenum),numel(prnlist))];
    end
    
    
%     % Load mean red values if they exist.
%     try
%     load([pathname 'mrv_and_pixel_area_excluding_redundant_pixels' TestName{1} PartName{partnum} '.mat'], 'meanRed', 't_datenum');
%     meanRedppaAllParts = [meanRedAllParts; meanRed]; % ntimes x nprns
%     t_datenumppaAllParts = [t_datenumAllParts, t_datenum]; % 1 x ntimes
%     catch
%         t_datenumppaAllParts = [t_datenumAllParts, t_datenum]; % 1 x ntimes
%         meanRedppaAllParts = [meanRedAllParts; nan*zeros(numel(t_datenum),numel(prnlist))];
%     end
    % Load mean red values if they exist.
    try
        load([pathname 'normed_mrv_and_FZpixel_area_excluding_redundant_pixels' ...
            TestName{1} PartName{partnum} '.mat'], 'meanRed', 't_datenum', 'pixelArea');
        pixelAreaAllParts = [pixelAreaAllParts; pixelArea];
        meanRedAllParts = [meanRedAllParts; meanRed]; % ntimes x nprns
        t_datenumAllParts = [t_datenumAllParts, t_datenum]; % 1 x ntimes
    catch
        t_datenumAllParts = [t_datenumAllParts, t_datenum]; % 1 x ntimes
        meanRedAllParts = [meanRedAllParts; nan*zeros(numel(t_datenum),numel(prnlist))];
        pixelAreaAllParts = [pixelAreaAllParts; nan*zeros(numel(t_datenum),numel(prnlist))];
    end

%     % Try to plot just Part 2 SR for PRN 26.
%     if partnum == 2
%         prn_idx = 3;
%     % Require the direct LOS of PRN 26 to be below the
%     % antenna horizon (phi > 90 deg).
%     arearows = ...
%         find(isfinite(deltaSRAllParts(:,prn_idx)) & ...
%         phi_svAllParts(:,prn_idx) > 90);
%     if ~isempty(arearows)
%         figure(1)
%         h(prn_idx) = plot(sr_datenumAllParts(arearows), ...
%             deltaSRAllParts(arearows,prn_idx), ...
%             '.','MarkerSize', 14);
%         datetick('x', 15)
%         ylabel(['' ...
%             '\Delta SR (dB)'])
%         title(['Time series of relative surface reflectivity (SR) via reflected GNSS power ' ...
%             '\newline for PRN ' num2str(prnlist(prn_idx))])% ' and optical mean red value (MRV) over GNSS-R Fresnel zone']);
%         keyboard
%         grid on
%         set(gca, 'FontSize', 14)
%         print(gcf,'-dpdf', [pathname filesep 'dsr_vs_time_prn' num2str(prnlist(prn_idx)) '.pdf']) % Save figure as an image
%     end
end


% Compute the pixel area.
% area = meanRedAllParts./meanRedppaAllParts;
% srppa = deltaSRAllParts./FZAreaAllParts;
fractionalArea = pixelAreaAllParts./FZAreaAllParts;
figure(1)
plot(t_datenumAllParts, FZAreaAllParts,'LineWidth',2);
hold on
plot(t_datenumAllParts, pixelAreaAllParts,':','LineWidth',2);
legend([num2str(prnlist'); num2str(prnlist')])
ax = axis;
axis([ax(1:2) 0 10])
datetick('x', 15)

% Plot the mean red value over time for all parts.
for prn_idx = 1:numel(prnlist)
    
    % Require the whole FZ to be imaged, and the direct LOS to be below the
    % antenna horizon (phi > 90 deg).
    arearows = ...
        find(isfinite(deltaSRAllParts(:,prn_idx)) & ...
        isfinite(meanRedAllParts(:,prn_idx)) & ...
        phi_svAllParts(:,prn_idx) > 90 & ...
        fractionalArea(:,prn_idx) >= 0.99);% & ...

    if ~isempty(arearows)
        rho = corrcoef(deltaSRAllParts(arearows,prn_idx), meanRedAllParts(arearows, prn_idx))
        figure(prn_idx)
        subplot(211)
        yyaxis left
    h(prn_idx) = plot(sr_datenumAllParts(arearows), deltaSRAllParts(arearows,prn_idx), ...
        '.','MarkerSize', 14);
    datetick('x', 15)
    ylabel(['' ...
        '\Delta SR (dB)'])
    title(['(a) Time series of relative surface reflectivity (SR) via reflected GNSS power ' ...
        '\newline for PRN ' num2str(prnlist(prn_idx)) ' and optical mean red value (MRV) over GNSS-R Fresnel zone']);
set(gca, 'FontSize', 14)


%     subplot(221)
%     h(prn_idx) = plot(sr_datenumAllParts(arearows), deltaSRAllParts(arearows,prn_idx), ...
%         '.','MarkerSize', 14);
%     grid on
%     % Plot HH:MM on the horizontal axis.
%     datetick('x', 15)
%     ylabel(['' ...
%         'Delta Surface Reflectivity (dB)'])
%     title(['Surface reflectivity based on reflected power ' ...
%         'for PRN ' num2str(prnlist(prn_idx))]);
%     
%     figure(prn_idx)
%     subplot(223)
    yyaxis right
    h(prn_idx) = plot(t_datenumAllParts(arearows), meanRedAllParts(arearows,prn_idx),...
        '.', 'MarkerSize', 14);
%     xlabel(['UT HH:MM on ' datestr(t_datenum(1), 'mmm dd, yyyy')])
%     grid on
%     % Plot HH:MM on the horizontal axis.
%     datetick('x', 15)
    ylabel('MRV')
    grid on
set(gca, 'FontSize', 14)
% % Plot vertical lines at relevant time points.
% ax = axis;
% plot([datenum([2020 2 21 17 58 0]), datenum([2020 2 21 17 58 0])], ...
%     ax(3:4), 'k');
% 
% plot([datenum([2020 2 21 18 43 0]), datenum([2020 2 21 18 43 0])], ...
%     ax(3:4), 'k');
%     title(['Mean red value of all camera pixels within the Fresnel zone ' ...
%         'for PRN ' num2str(prnlist(prn_idx))]);
% 
    subplot(212)
    scatter(meanRedAllParts(arearows,prn_idx), ...
        deltaSRAllParts(arearows,prn_idx), 'filled', 'MarkerFaceColor','black')
    xlabel('Optical Mean Red Value')
    ylabel('GNSS-R \Delta SR (dB)')
    leg = legend(['\rho = ' num2str(rho(1,2))], 'Location', 'best');
    set(leg, 'FontSize', 14)
    grid on
    title(['(b) Relative GNSS-R surface reflectivity ' ...
        'for PRN ' num2str(prnlist(prn_idx)) ...
        ' versus \newline optical mean red value over GNSS-R Fresnel zone'])
set(gca, 'FontSize', 14)
% 
%     subplot(224)
%     scatter(FZAreaAllParts(arearows,prn_idx), deltaSRAllParts(arearows,prn_idx))
%     xlabel('FZ area [m^2]')
%     ylabel('GNSS-R \Delta Surface Reflectivity')
%     FZrho = corrcoef(deltaSRAllParts(arearows,prn_idx), FZAreaAllParts(arearows, prn_idx));
%     legend(['\rho = ' num2str(FZrho(1,2))]);
    
%     if saveFigs==1 % Check if Saving figures is requested
%         figsav1 = [figsavPath filesep num2str(get(gcf,'Number'))]; % path to save this figure, in folder with figure number
%         if ~exist(figsav1,'dir')
%             mkdir(figsav1);
%         end % Create the save directory if it doesn't exist
h=gcf;
% set(h,'PaperPositionMode','auto'); 
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% set(h,'Position',[50 50 1200 800]);
% set(h,'PaperPosition', [1 1 28 19]);
print(gcf,'-dpdf', [pathname filesep 'dsr_vs_mrv_prn' num2str(prnlist(prn_idx)) '.pdf']) % Save figure as an image
% saveas(gcf,[pathname filesep 'dsr_vs_mrv_prn' num2str(prnlist(prn_idx)) '.pdf']) % Save figure as an image
    end %if ~isempty(arearows)
%     end


end
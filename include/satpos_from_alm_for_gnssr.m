function [t_gps, prn_gps, vecr_svfromR0_ned] = satpos_from_alm_for_gnssr(t_0, t_f, time_step_sec, latR0, lonR0, htR0)
% function vecr_svfromR0_ned = satpos_from_alm_for_gnssr(t_0, latR0, lonR0, htR0)
% returns a vector in NED coordinates of satellites from an origin R0,
% whose latR0, lonR0 are in degrees, and htR0 in m.  t_0 is a datenum array
% of the utc times.  This function uses the Constellation toolbox.

[year, ~, ~] = datevec(t_0);

doy = floor(t_0 - datenum([year, 0, 0]));

almanac_directory = ['/Users/seebany/Dropbox/Data/almanacs/' num2str(year) filesep];
    alm_file=[almanac_directory, num2str(doy, '%03d') '.ALM'];

    % Compute satellite azimuth and elevations over time.
    % These steps follow the procedure of yuma2az_el.m but account for week
    % rollovers better.
    % convert the station location from lat, long, alt. to ECEF vector
    llh_reflect = [latR0*pi/180, lonR0*pi/180, htR0];
    ecef_reflect = lla2ecef(llh_reflect);

    t_0_gpst = utc2gps(datevec(t_0));
    rollover_flag = floor(t_0_gpst(1)/1024);
    % load the GPS almanac for the given almanac week
    alm_2_use = readyuma(alm_file, rollover_flag);
    % convert the almanacs to ephemeris format
    [gps_ephem] = alm2geph(alm_2_use);
    % first convert the start and stop times to GPS time.
    start_gps = utc2gps(datevec(t_0));
    stop_gps = utc2gps(datevec(t_f));

    % compute satellite positions in ECEF frame for the given time range and interval
    [t_gps,prn_gps,x_gps,v_gps] = propgeph(gps_ephem, start_gps, stop_gps, time_step_sec);

    % compute LOS vectors in ECEF frame
    [t_los_gps, gps_los, los_ind] = los(t_gps(1,:), ecef_reflect, t_gps, [prn_gps x_gps]);

    % convert LOS in ECEF to NED frame
    vecr_svfromR0_ned = ecef2ned(gps_los, llh_reflect);

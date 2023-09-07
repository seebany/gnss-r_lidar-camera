function phi = compute_phi(vecr_svfromR0_ned, ...
        kappa, BoomHead, prnlist, t_datenum, az_el_table)
% phi = compute_phi_of_sv(TestName{1}, PartName{partnum}, vecr_svfromR0_ned, ...
%        kappa, BoomHead, t_datenum, az_el_table);
% computes the reflected antenna zenith angle of the direct line-of-sight
% from the sv to antenna, in degrees.
% Input vecr_svfromR0_ned is n x 3 in NED meters.
%
% Seebany Datta-Barua
% 2 Jan 2023

% Reorder input position vectors from NED to ENU.
vecr_svfromR0_enu = [vecr_svfromR0_ned(:,2), vecr_svfromR0_ned(:,1), ...
    - vecr_svfromR0_ned(:,3)];
dist = sqrt(sum(vecr_svfromR0_enu.^2,2));

% Loop over time and prns.
for t_idx = 1:numel(t_datenum)
    for prn_idx = 1:numel(prnlist)
            % Reorder the vector to be ENU coordinates.
            rows = find(az_el_table(:,1) == t_datenum(t_idx) & ...
                az_el_table(:,3) == prnlist(prn_idx));
            if isempty(rows) keyboard; end
            % Compute the distance in m.
            Rtr(t_idx, prn_idx) = dist(rows);
            %--------------Compute antenna gain Gr----------------------------------/

            % First we need to know angle phi between the reflection antenna's
            % \hat{y}_r and vecr_svfromR0.
            %
            vecr_svfromR0_tbu = rot_tbu_from_enu(vecr_svfromR0_enu(rows,:), BoomHead);

            % The rotation is the same as for the lidar
            % so reuse that code, passing kappa instead of psi_L.
            vecr_svfromR0_xyzR = rot_xyzL_from_tbu(vecr_svfromR0_tbu, kappa);

            phi(t_idx, prn_idx) = acosd(vecr_svfromR0_xyzR(:,2)./Rtr(t_idx, prn_idx)); % deg

    end % for prn_idx
end % for t_idx


function vecr_PCfromL0_enu = rot_enu_from_tbu(vecr_PCfromL0_tbu, BoomHead)
% function vecr_PCfromR0_enu = rot_enu_from_tbu(vecr_PCfromL0_tbu, BoomHead)
% rotates an array of vectors from boom coordinates [transverse,
% boom-aligned, up] to [east, north, up] components given the boom heading
% angle in degrees. Input vec is nx3. BoomHead is scalar deg. Output is n x 3.

% Rotate Pixel point cloud from Boom to ENU coords
ERB = [cosd(BoomHead)   sind(BoomHead)  0;              % Rotation matrix Boom to ENU
      -sind(BoomHead)   cosd(BoomHead)   0;
      0         0         1];
vecr_PCfromL0_enu = (quicktimes(ERB,vecr_PCfromL0_tbu'))'; % Apply rotation to get pixel point cloud in ENU coords

function vecr_PCfromL0_tbu = rot_tbu_from_enu(vecr_PCfromL0_enu, BoomHead)
% function vecr_PCfromR0_tbu = rot_tbu_from_enu(vecr_PCfromL0_enu, BoomHead)
% rotates an array of vectors from [east, north, up] components to boom 
% coordinates [transverse, boom-aligned, up] given the boom heading
% angle in degrees (azimuthal angle defined positive clockwise from north). 
% Input vec is nx3. BoomHead is scalar deg. Output is n x 3.

% Rotate Pixel point cloud from Boom to ENU coords
ERB = [cosd(BoomHead)   sind(BoomHead)  0;              % Rotation matrix Boom to ENU
      -sind(BoomHead)   cosd(BoomHead)   0;
      0         0         1];
BRE = inv(ERB);
vecr_PCfromL0_tbu = (quicktimes(BRE,vecr_PCfromL0_enu'))'; % Apply rotation to get pixel point cloud in ENU coords

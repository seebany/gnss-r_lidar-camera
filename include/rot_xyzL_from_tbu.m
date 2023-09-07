% Rotate Pixel point cloud from Boom coords to Lidar coords
% SDB believes it also works to rotate a point from boom coords to
% reflection antenna coords.
function vecr_PPCfromC0_xyzL = rot_xyzL_from_tbu(vecr_PPCfromC0_tbu, Lid_el)

BRL = [1    0               0;                  % Rotation matrix Lidar to Boom
      0     cosd(Lid_el)	-sind(Lid_el);
      0     sind(Lid_el)    cosd(Lid_el)];

vecr_PPCfromC0_xyzL = (quicktimes(inv(BRL),vecr_PPCfromC0_tbu'))';

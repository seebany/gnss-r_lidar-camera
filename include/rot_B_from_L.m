function vecr_PCfromC0_tbu = rot_tbu_from_xyzL(vecr_PCfromC0_xyzL, Lid_el)
% rotate from Lidar to Boom coords

BRL = [1    0               0;                  % Rotation matrix Lidar to Boom
      0     cosd(Lid_el)	-sind(Lid_el);
      0     sind(Lid_el)    cosd(Lid_el)];
vecr_PCfromC0_tbu = quicktimes(BRL,vecr_PCfromC0_xyzL'); % Apply rotation to get point cloud in Boom coords
vecr_PCfromC0_tbu = vecr_PCfromC0_tbu';

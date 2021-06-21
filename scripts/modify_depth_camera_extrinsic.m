% 旧旋转参数, [yaw pitch roll]
eul_old = [0.017619901402921023 -1.9147626638120656 2.7351103868502604];
eMc = eul2rotm(eul_old);
% 调整角度degree
delta_pitch = -0.9;
eMr = eul2rotm([0, delta_pitch*pi/180, 0]);

eMc = eMr * eMc;
% 新旋转参数, [yaw pitch roll]
eul_new = rotm2eul(eMc)

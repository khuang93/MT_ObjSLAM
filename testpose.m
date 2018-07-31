clear;
clc;
quat_cb=quaternion(0.5 ,-0.5 ,0.5 ,-0.5)
quat1=quaternion(0.93579, 0.0281295, 0.0740478, -0.343544)
trans1_b=[-0.684809 1.59021 0.91045]'
T_bw_1=[quat2rotm(quat1), quat2rotm(quat1)*trans1_b;0 0 0 1]
T_cb=[quat2rotm(quat_cb),[0 0 0]';[0 0 0 1]]
T_cw_1=T_bw_1*T_cb
T_wc_1=inv(T_cw_1)

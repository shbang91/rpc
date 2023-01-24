function mat = quat2mat(quat)
%QUAT2MAT Transforms a quaternion to a rotation matrix

    quat = quat(:);
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    mat = [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w; ...
        2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w; ...
        2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                 CONTRAST MAXIMISATION BRANCH AND BOUND
%
%
% This package contains the source code which implements the
% Contrast maximisation BnB algorithm (CMBnB) in
%
%       Globally Optimal Contrast Maximisation for Event-based  
%                       Motion Estimation
%
% The source code, binaries and demo are supplied for academic use only.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Rotation = Axis2R(axis_angle)
    angle = norm(axis_angle);
    axis = axis_angle/angle;
    kz = axis(3);
    ky = axis(2);
    kx = axis(1);
    K = [0 -kz ky; kz 0 -kx; -ky kx 0];
    Rotation = eye(3) + sin(angle)*K + (1-cos(angle))*K*K;
end


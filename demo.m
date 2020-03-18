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

clear all
close all

addpath matlab_functions
addpath mex


% Read data
format long
path = 'data';
savepath = 'data';
[event,K] = read(path);
seq = event.seq;
intrinic = [K(1) 0 0; 0 K(2) 0; K(3) K(4) 1]';
radial = [K(5) K(6) K(9)];
tang = [K(7) K(8)];
CameraPar = cameraParameters('IntrinsicMatrix',intrinic,...
    'RadialDistortion',radial,'TangentialDistortion',tang);
parameter
IK = CameraPar.IntrinsicMatrix;

% run methods
plot_input;
drawnow
CM;
bnb_all;
plot_result;

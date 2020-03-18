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

options = optimoptions(@fmincon,'StepTolerance',1.0000e-6,'ConstraintTolerance',1.0000e-6,'OptimalityTolerance',1.0000e-6,'Display','iter','SubproblemAlgorithm','cg');
A = [];
b = [];
Aeq = [];
Beq = [];
tic;
[CMGD,~] = fmincon(@(x) minfun(x,seq,CameraPar,1,0),x0,A,b,Aeq,Beq,lb,ub,[],options);
CM_time = toc;
reg_points_CMGD = registration(seq,CMGD,CameraPar,1,0);
[contrast_CMGD,event_image_CMGD] = cal_contrast_nt(reg_points_CMGD,1); 

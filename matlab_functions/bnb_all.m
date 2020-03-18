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

ray = correction(seq,CameraPar,0,0);
seq(:,1) = seq(:,1) - seq(1,1);
tic;
[a,c]=rot3_matches(seq,lb,ub,IK,ray);
bnb_time = toc;
C = rotm2axang(c);
BNB_C = C(1:3)*C(4);
reg_points_BNB = registration(seq,BNB_C,CameraPar,1,0);
[contrast_BNB,event_image_BNB] = cal_contrast_nt(reg_points_BNB,1);  

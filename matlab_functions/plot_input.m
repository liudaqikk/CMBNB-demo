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

%create red and blue mask (p = 1 and p = 0)
maskr = seq(:,4) == 0;
maskb = seq(:,4) == 1;
seqr = seq(maskr,:);
seqb = seq(maskb,:);
[input_contrast,~] = cal_contrast_nt(seq(:,2:4),1); 

figure('Name','Input','Position', [10 10 600 800])

subplot(2,1,1);
plot3(seqr(:,1),seqr(:,2),seqr(:,3),'r.');
hold on
plot3(seqb(:,1),seqb(:,2),seqb(:,3),'b.');
hold off
title('Event stream')

subplot(2,1,2);
hold on
plot(seqr(:,2),180-seqr(:,3),'r.');
plot(seqb(:,2),180-seqb(:,3),'b.');
title(sprintf('Event image without motion compensation (contrast = %.3f)',input_contrast));
axis equal

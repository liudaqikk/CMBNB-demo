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


figure('Name','Output','Position', [610 10 900 800])

% plot event image
title('Results')
subplot(2,2,1);
imagesc(event_image_BNB);
title(sprintf('CMBnB event image (contrast = %.3f)',contrast_BNB));
subplot(2,2,2);
imagesc(event_image_CMGD);
title(sprintf('CMGD event image (contrast = %.3f)',contrast_CMGD));


%create red and blue points
BNBr = reg_points_BNB(maskr,:);
BNBb = reg_points_BNB(maskb,:);

CMGDr = reg_points_CMGD(maskr,:);
CMGDb = reg_points_CMGD(maskb,:);

subplot(2,2,3);
hold on
plot(BNBr(:,1),180-BNBr(:,2),'r.');
plot(BNBb(:,1),180-BNBb(:,2),'b.');
title(sprintf('CMBnB (contrast = %.3f)',contrast_BNB))

subplot(2,2,4);
hold on
plot(CMGDr(:,1),180-CMGDr(:,2),'r.');
plot(CMGDb(:,1),180-CMGDb(:,2),'b.');
title(sprintf('CMGD (contrast = %.3f)',contrast_CMGD))

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

function [contrast,event_img] = cal_contrast_nt(points,done)
    format long
    rows = 180;
    cols = 240;
    event_img = zeros(rows,cols);
    for i = 1 : size(points,1)
%         distx = abs(points(i,2)  - x);
%         [~,ver] = min(distx);
%         disty = abs(points(i,1)  - y);
%         [~,hor] = min(disty);
%         if hor<=cols && ver>=1 && hor>=1 && ver<=rows           
%             event_img(ver,hor) = event_img(ver,hor)+1;
%         end
        ver = round(points(i,2));
        hor = round(points(i,1));
        dis = sqrt((points(i,2)-ver)^2 + (points(i,1)-hor)^2);
        if hor<=cols && ver>=1 && hor>=1 && ver<=rows
            event_img(ver,hor) = event_img(ver,hor) + 1;
        end
    end
    event_img = reshape(event_img,43200,1);
    contrast = event_img'*event_img/43200;
    if done == 1
        event_img = reshape(event_img,180,240);
    end
end


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

function [contrast,event_img] = cal_contrast_tg(points,done)
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
        if hor<cols && ver>1 && hor>1 && ver<rows
            x = hor-1:hor+1;
            y = ver-1:ver+1;
            [X, Y] = meshgrid(x,y);
            T = [X(:) Y(:)];
            value = reshape(normpdf(sqrt(sum((points(i,1:2) - T).^2,2)),0,1),3,3);
            event_img(ver-1:ver+1,hor-1:hor+1) = event_img(ver-1:ver+1,hor-1:hor+1) + value;
        end
    end
    event_img = imgaussfilt(event_img, 1.);
    event_img = reshape(event_img,43200,1);
    contrast = event_img'*event_img/43200  - (ones(1,43200)*event_img/43200)^2;
    if done == 1
        event_img = reshape(event_img,180,240);
    end
        
end


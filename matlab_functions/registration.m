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

function reg_points = registration(event,angle,CameraPar,time_interval,translation)   
    format long
    % translation correction
    %backproject = (CameraPar.IntrinsicMatrix')^(-1)*[event(:,2:3) ones(size(event,1),1)]';
    backproject = correction(event,CameraPar,translation,time_interval);    
    
    reg_points = zeros(size(event,1),3);
    reg_points(1,:) = [event(1,2) event(1,3) event(1,4)];
    
    %convert each event back to reference frame
    for i = 2 : size(event,1)
        a = angle .* (event(i,1)-event(1,1))/(event(end,1)-event(1,1));
%         R = rotationVectorToMatrix(a);
        R = Axis2R(a);
        if event(i,1)-event(1,1) == 0
            R = eye(3);
        end
        project = CameraPar.IntrinsicMatrix*R*backproject(:,i);
        
        % for star data set
        reg_points(i,:) = [project(1)/project(3) project(2)/project(3) event(i,4)];
        
        % for shape data set
        % reg_points(i,:) = [project(1)/project(3) project(2)/project(3) event(i,4)];
    end
%     reg_points = round(reg_points);
    
end

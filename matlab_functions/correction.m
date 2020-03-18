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

function backproject = correction(event,CameraPar,translation,time_interval)
    backproject = (CameraPar.IntrinsicMatrix)^(-1)*[event(:,2:3) ones(size(event,1),1)]';
%     time_array = event(:,1) - event(1,1);
%     backproject = backproject - ((translation.*(time_array))./time_interval)';
    backproject_norm = sqrt(sum(backproject.*backproject,1));
    backproject = backproject ./ backproject_norm;
end


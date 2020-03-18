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

% Compile mex functions
clc
CXXFLAGS = "\$CXXFLAGS -Wall-std=c++11 -fopenmp";
close all;


% Create directory to save binaries
mkdir('mex')

mex -v CXXFLAGS="\$CXXFLAGS -Wall -std=c++11 -fopenmp" CXXOPTIMFLAGS='\$CXXOPTIMFLAGS -Ofast -DNDEBUG' LDOPTIMFLAGS="$LDOPTIMFLAGS -fopenmp -O2" -lgomp...
    -lmwblas -Iinclude/ -Ithird_party/eigen3 ...
    -output mex/rot3_matches...
    src/rot3_matches.cpp ...
    src/geometry.cpp ...
    src/bnb_rsearch_3dof_matches.cpp 

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <iostream>
#include "registration.h"
#include "mex.h"

using namespace reg;
using namespace reg::search;
using namespace std;
using namespace Eigen;


double *normalisecols(const double *X, size_t n)
{
    double *outArray = (double *)mxMalloc(3*n*sizeof(double));

    double norm;
    int i,j;
    for(j=0; j<n; j++) /* Compute a matrix with normalized columns */
    {
        norm = 0.0;
        for(i=0; i<3; i++)
        {
            norm += (X[i + 3*j])*(X[i + 3*j]);
        }
        norm = sqrt(norm);
        for(i=0; i<3; i++)
        {
            outArray[i + 3*j] = X[i + 3*j]/norm;
        }
    }
    return outArray;
}


// Input
#define X_IN     prhs[0]
#define LW_B     prhs[1]
#define UP_B     prhs[2]
#define CA_K     prhs[3] //
#define RAYS     prhs[4]

// Output
#define Q_OUT    plhs[0]  // Solution quality
#define R_OUT    plhs[1]  // Optimal rotation

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check mex-arguments
    if (nlhs != 2)
    {
        mexErrMsgTxt("Invalid number of output variables. Expected 2 LHS vars.");
    }

    if (mxGetN(X_IN) != 4)
    {
        mexErrMsgTxt("Invalid input. Expected 4xn matrices.");
    }

    // Retrieve input
    const size_t numOfPts = mxGetM(X_IN);


    double *X = mxGetPr(X_IN);
    double *LB = mxGetPr(LW_B);
    double *UB = mxGetPr(UP_B);
    double *K = mxGetPr(CA_K);
    double *RA = mxGetPr(RAYS);
    MatrixXd matrixX = Map<MatrixXd> (X,numOfPts,4);
    VectorXd lowerB = Map<VectorXd> (LB,3);
    VectorXd upperB = Map<VectorXd> (UB,3);
    MatrixXd MK = Map<MatrixXd> (K,3,3);
    MatrixXd tray = Map<MatrixXd> (RA,3,numOfPts);
    MatrixXd ray = tray.transpose();


    size_t q = 0;
    AxisAngle resp;

    q = rot3_matches(matrixX, lowerB, upperB, ray, MK, resp);

    Matrix3 R;
    fromAxisAngle(R,resp);

    R_OUT = mxCreateDoubleMatrix(3, 3, mxREAL);
    std::copy(R.getPr(), R.getPr()+9, mxGetPr(R_OUT));

    Q_OUT = mxCreateDoubleScalar(q);
}

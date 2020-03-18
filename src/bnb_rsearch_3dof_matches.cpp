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

#include "reg_common.h"
#include "search.h"
#include "registration.h"
#include "assert.h"
#include "state_priority_hashtable.h"
#include "matches_indexation.h"
#include "euc_matches_indexation.h"
#include <iostream>

using namespace reg::search;
using namespace std;
using namespace Eigen;


// ang distance

int reg::search::rot3_matches(const MatrixXd &X, VectorXd &lowerB, VectorXd &upperB,
                              MatrixXd &ray, const MatrixXd MK, AxisAngle &result)
{
    typedef RotationSearchSpaceRegion3DOFS8 SSR;

    SSR guessAndResult(lowerB(0), lowerB(1), lowerB(2), upperB(0), upperB(1), upperB(2));
    DataIndexation<SSR> *dsi0 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi1 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi2 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi3 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi4 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi5 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi6 = new MatchesIndexation<SSR>(X,ray, MK);
    DataIndexation<SSR> *dsi7 = new MatchesIndexation<SSR>(X,ray, MK);
    //cout<<dsi->box<<endl;
    //std::cout<<"sweep out num of pts = "<< rempts<<std::endl;

    //mxAssert(rempts>0, "rempts>0");

    const int buckets = MAX(10, (int)X.cols()/10);
    //int gap = 10000; // for boxs and dynamic and poster
    int gap = 5000; //for shapes
    int lwbnd = 0;
    int qual = bnb_search_table<SSR,8>(*dsi0,*dsi1,*dsi2,*dsi3,*dsi4,*dsi5,*dsi6,*dsi7,
                                      lwbnd, gap, buckets,guessAndResult);

    delete dsi0;
    delete dsi1;
    delete dsi2;
    delete dsi3;
    delete dsi4;
    delete dsi5;
    delete dsi6;
    delete dsi7;
    result = centre(guessAndResult);
    return qual;
}

//Match list
/*int reg::search::rot3_matches_ml(const Matrix3X &X, const Matrix3X &Y,
                                 double th, int gap, int lwbnd, AxisAngle &result)
{
    typedef RotationSearchSpaceRegion3DOFS8 SSR;

    SSR guessAndResult(-PI, -PI, -PI, PI, PI, PI);
    DataIndexation<SSR> *dsi = new MatchesIndexation<SSR>(X, Y, th);
    dsi->sweep();

    const int buckets = MAX(10, (int)X.cols()/10);
    const int qual = bnb_search_ml_table<SSR,8>(*dsi, lwbnd, gap, buckets,
                                                guessAndResult);
    delete dsi;
    result = centre(guessAndResult);
    return qual;
}


// l2 distance


int reg::search::rot3_matches_l2(const Matrix3X &X, const Matrix3X &Y,
                                 double th, int gap, int lwbnd, AxisAngle &result)
{
    mxAssert(gap>=0, "incorrect gap");
    mxAssert(lwbnd>=0, "incorrect lower bound");

    mxAssert(X.n>0, "num of pts must be >0 ");
    typedef RotationSearchSpaceRegion3DOFS8 SSR;

    SSR guessAndResult(-PI, -PI, -PI, PI, PI, PI);
    DataIndexation<SSR> *dsi = new EucMatchesIndexation<SSR>(X, Y, th);
    size_t n = dsi->sweep();

    std::cout<<"sweep size = "<<n<<std::endl;

    const int buckets = MAX(10, (int)X.cols()/10);
    int qual = bnb_search_table<SSR,8> (*dsi,lwbnd,gap,buckets,guessAndResult);

    std::cout<<" >> found qual = "<<qual<<std::endl;


    delete dsi;

    std::cout<<" >> after delete dsi = "<<std::endl;

    result = centre(guessAndResult);
    return qual;
}


int reg::search::rot3_matches_l2_ml(const Matrix3X &X, const Matrix3X &Y,
                                    double th, int gap, int lwbnd, AxisAngle &result)
{
    typedef RotationSearchSpaceRegion3DOFS8 SSR;

    SSR guessAndResult(-PI, -PI, -PI, PI, PI, PI);
    DataIndexation<SSR> *dsi = new EucMatchesIndexation<SSR>(X,Y,th);
    dsi->sweep();

    const int buckets = MAX(10, (int)X.cols()/10);
    int qual = bnb_search_ml_table<SSR,8>(*dsi,lwbnd,gap,buckets,guessAndResult);

    delete dsi;
    result = centre(guessAndResult);
    return qual;
}*/

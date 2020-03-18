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

#ifndef MATCH_DATASET_INDEXATION_
#define MATCH_DATASET_INDEXATION_

#include "reg_common.h"
#include "data_indexation.h"
#include "state.h"

namespace reg {
namespace search { 

template<class SSR>
class EucMatchesIndexation : public DataIndexation<SSR>
{
public:
    EucMatchesIndexation( const Matrix3X &X, const Matrix3X &Y, double th );
    ~EucMatchesIndexation();

    int sweep();
    int sweep(int *matchList, int matchListSize, std::vector<bool> &matches);

    int size() const;

    int evalUpperBound(SSR ssr, int lwbnd) const;
    int evalUpperBound(SSR ssr, int lwbnd,
                       int *matchList, int matchListSize,
                       std::vector<bool> &matches) const;

    int evalLowerBound(SSR ssr) const;
    int evalLowerBound(SSR ssr, int *matchList, int matchListSize) const;
    int evalLowerBound(SSR ssr, int *matchList, int matchListSize,
                       std::vector<bool> &matches) const;


private:

    const Matrix3X &X_in;
    const Matrix3X &Y_in;
    const double th;

    // contains unit-norm vectors
    Matrix3X X, Y;
    
    // equivalent ang. errors
    Vector TH;

    int _size;
    int offset;
};


} // End namespace sarch
} // End namespace reg

#include "euc_matches_indexation.hpp"


#endif


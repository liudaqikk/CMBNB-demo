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

#ifndef REG_DATA_INDEXATION_
#define REG_DATA_INDEXATION_

#include <vector>
using namespace Eigen;
namespace reg {
namespace search {

template<class SSR>
class DataIndexation
{
public:
    virtual ~DataIndexation(){}

    virtual int evalUpperBound(SSR ssr) const = 0;
    virtual int evalLowerBound(SSR ssr) const = 0;
    virtual void initialization(SSR ssr);
    virtual void contrast_bound(SSR ssr, int bound_required);
    virtual void warp_event(int bound_required);
    virtual void rotate();
    virtual void compute_circle(ArrayXd temp_box);
    virtual void Axis2R(MatrixXd temp_c);
    virtual void pixUcir(int i, double lx1, double ly1, double ux1, double uy1);
    virtual ArrayXXi midptellipse(double rx, double ry, double xc, double yc,int row, int col);
    virtual int update_image(int y, int x, int lx, int ly, ArrayXXi im);
    virtual void compute_localmax(int lm);
    //virtual int size() const = 0;

};


inline double dist_sq( Vector3 &a1, double *a2)
{
  double dist_sq = 0, diff;
  for (int i=0; i<3;i++)
  {
    diff = (a1[i] - a2[i]);
    dist_sq += diff*diff;
  }
  return dist_sq;
}


} // End namespace sarch
} // End namespace reg

#endif

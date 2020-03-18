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

#ifndef MATCHES_INDEXATION_
#define MATCHES_INDEXATION_

#include "reg_common.h"
#include "data_indexation.h"
#include "state.h"

namespace reg {
namespace search {

template<class SSR>
class MatchesIndexation : public DataIndexation<SSR>
{
public:

    MatchesIndexation( const MatrixXd &X, const MatrixXd &rays, const MatrixXd &MK);
    MatchesIndexation();
    int evalUpperBound(SSR ssr) const;

    int evalLowerBound(SSR ssr) const;
    int itr = 0;
    void initialization(SSR ssr);
    void warp_event(int bound_required);
    void Axis2R(MatrixXd temp_c);
    void warp_time();
    void rotate();
    void compute_circle(ArrayXd temp_box);
    void pixUcir(int i, double lx, double ly, double ux, double uy);
    ArrayXXi midpointcircle(double r, double xc, double yc, int row, int col);
    void compute_localmax(int lm);
    ArrayXXi midptellipse(double rx, double ry, double xc, double yc,int row, int col);
    int update_image(int y, int x, int lx, int ly, ArrayXXi im);

private:

    const MatrixXd &event;
    const MatrixXd &ray;
    const MatrixXd &intrinic;
    ArrayXXd upper_image;
    MatrixXd box;
    double upper_bound;
    double lower_bound;
    MatrixXd M;
    int _size;
    void contrast_bound(SSR ssr, int bound_required);
    int event_length;    // the number of event to process every time
    double bound;        // radius of the rotation box
    VectorXd c;          // center of the rotation box
    int rows, cols;      // size of the event image
    ArrayXXd reg_points; // the coordinates of the wapred event
    ArrayXXd cr;         // circle coordinate and radius
    ArrayXd r00, r01, r02, r10, r11, r12, r20, r21, r22;
    ArrayXd rotate_rx, rotate_ry, rotate_rz;
    double upper_contrast;
    int global_count;
    ArrayXd localmax;
    int global_max;
    int round_require;
    int ellipse_require;
    double radius_limit;
};


} // End namespace sarch
} // End namespace reg

#include "matches_indexation.hpp"


#endif

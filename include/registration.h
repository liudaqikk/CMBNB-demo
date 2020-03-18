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

#ifndef REG_REG_SEARCH_
#define REG_REG_SEARCH_

#include "reg_common.h"
#include "state.h"

using namespace Eigen;

//TODO: split accordig to papers.

namespace reg {
    namespace search {

        //--------------------------------------------------------------------------
        //     Rotation search
        //--------------------------------------------------------------------------

        // on points

        int bnb_rsearch_3dof_mcirc(const MatrixXd &X, const Matrix3X &Y, double th,
                                   int gap,  int lwbnd, int buckets,
                                   AxisAngle &result);
        //rot3_ml
        int bnb_rsearch_3dof_mcirc_ml(const Matrix3X &X, const Matrix3X &Y, double th,
                                      int gap,  int lwbnd,  int buckets,
                                      AxisAngle &result);

        // on matches

        // ang distance
        int rot3_matches(const MatrixXd &X, VectorXd &lowerB, VectorXd &upperB, MatrixXd &ray,
                                   const MatrixXd MK, AxisAngle &result);

        int rot3_matches_ml(const Matrix3X &X, const Matrix3X &Y, double th,
                            int gap, int lwbnd, AxisAngle &result);



        // l2 distance
        int rot3_matches_l2(const Matrix3X &X, const Matrix3X &Y, double th,
                            int lwbnd, int gap, AxisAngle &result);

        int rot3_matches_l2_ml(const Matrix3X &X, const Matrix3X &Y, double th,
                               int lwbnd,int gap, AxisAngle &result);


        int rot1_matches_l2(const Matrix3X &X, const Matrix3X &Y, double th,
                            int lwbnd, int gap, double &result);

        int rot1_matches_l2_ml(const Matrix3X &X, const Matrix3X &Y, double th,
                               int lwbnd, int gap, double &result);


        // minmax
        double rot3minmax(const Matrix3X &X, const Matrix3X &Y, double gap, AxisAngle &result);


        //--------------------------------------------------------------------------
        //     Registration
        //--------------------------------------------------------------------------


        // 6 DoF with matches
        int reg6Matches(const Matrix3X &X, const Matrix3X &Y, double th,
                        int lwbnd, int gap, Transform3 &guessAndResult);

        int reg4Matches(const Matrix3X &X, const Matrix3X &Y, double th,
                        int lwbnd, int gap, Transform3 &guessAndResult);



        // Nested (6DoF)

        int nestedbnb_search_6dof_mcirc(const Matrix3X &X, const Matrix3X &Y, double th,
                                        int gap, int lwbnd,
                                        TranslationSearchSpaceRegion3DOF &trDom,
                                        int(*bnb_rsearch_3dof)(const Matrix3X&, const Matrix3X&,
                                                               double,int, int, int, AxisAngle&),
                                        Transform3 &guessAndResult,
                                        bool use_local_opt);

        int nestedbnb_search_6dof_mcirc_ml(const Matrix3X &X, const Matrix3X &Y, double th,
                                           int gap, int lwbnd,
                                           TranslationSearchSpaceRegion3DOF &trDom,
                                           Transform3 &guessAndResult,
                                           bool use_local_opt);


        // Local (6DoF)
        int local_search_6dof(const Matrix3X &X, const Matrix3X &Y, double th,
                              int lwbnd,
                              TranslationSearchSpaceRegion3DOF trDom,
                              int(*bnb_rsearch_3dof)(const Matrix3X&, const Matrix3X&,
                                                     double,int, int, int, AxisAngle&),
                              Transform3 &guessAndResult);


    } // End namespace sarch
} // End namespace reg

#endif

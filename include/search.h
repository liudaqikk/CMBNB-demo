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

#ifndef REG_SEARCH_
#define REG_SEARCH_

#include "reg_common.h"
#include "data_indexation.h"

namespace reg {
    namespace search {

        /**
         * @brief Find optimal quality of a transformation
         * @param psi Indexation of point sets.
         * @param knownSolutionQual Known solution quality.
         * @param gap BnB stop gap.
         * @param guessAndResult Guess search region and final region such that
         *        upbnd-lwbnd<=gap
         * @return Quality of the central transform of the optimal region.
         */
        template <class SSR, unsigned int BRANCHING_FACTOR>
        int bnb_search_queue(const DataIndexation<SSR> &psi,
                             int lwbnd, int gap, SSR &guessAndResult);


        template <class SSR, unsigned int BRANCHING_FACTOR>
        int bnb_search_table( DataIndexation<SSR> &dsi0,
                              DataIndexation<SSR> &dsi1,
                              DataIndexation<SSR> &dsi2,
                              DataIndexation<SSR> &dsi3,
                              DataIndexation<SSR> &dsi4,
                              DataIndexation<SSR> &dsi5,
                              DataIndexation<SSR> &dsi6,
                              DataIndexation<SSR> &dsi7,

                             int lwbnd, int gap, int buckets,
                             SSR &guessAndResult);

        template <class SSR, unsigned int BRANCHING_FACTOR>
        int searchTableDF(const DataIndexation<SSR> &dsi,
                          int lwbnd, int gap, int buckets,
                          SSR &guessAndResult);


        template <class SSR, unsigned int BRANCHING_FACTOR>
        int bnb_search_ml_table(const DataIndexation<SSR> &dsi,
                                int lwbnd, int gap, int buckets,
                                SSR &guessAndResult);


    } // End namespace sarch
} // End namespace reg

#include "search.hpp"

#endif

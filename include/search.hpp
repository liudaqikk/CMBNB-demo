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

#include "state_priority_queue.h"
#include "state_priority_hashtable.h"
#include "data_indexation.h"
#include "geometry.h"
#include <iostream>
#include <omp.h>
using namespace std;
namespace reg
{
namespace search
{

template <class SSR, unsigned int BRANCHING_FACTOR>
int bnb_search_queue(const DataIndexation<SSR> &dsi,
                     int lwbnd, int gap,
                     SSR &guessAndResult)
{
    int i, np;
    int state_lwbnd, upbnd;

    int count_eval_ubbnd;
    int count_eval_lwbnd;

    SearchState<SSR> *state;
    SSR **ssr_array; //array of pointers

    // Initialise
    count_eval_ubbnd = count_eval_lwbnd = 0;

    upbnd = dsi.evalUpperBound(guessAndResult, lwbnd);
    count_eval_ubbnd++;

    // No better solution than the known one in provided SSR
    mxAssert( (((lwbnd==0) &&(upbnd>=lwbnd)) || (lwbnd>0)), "Bound error");
    if (upbnd-lwbnd <= gap)
    {
        return lwbnd;
    }

    StatePriorityQueue<SSR> queue;


    ssr_array = new SSR*[BRANCHING_FACTOR];
    state = new SearchState<SSR>(guessAndResult, upbnd);
    queue.push(state);

    int iter = 0;
    while (queue.size())
    {
        iter++;

        // Find the state with the highest upper bound
        state = queue.pop();

        // Evaluate lower boud

        state_lwbnd = dsi.evalLowerBound( state->ssr );
        count_eval_lwbnd++;

        // Update solution
        if (state_lwbnd > lwbnd)
        {
            lwbnd = state_lwbnd;
            guessAndResult = state->ssr;
            queue.prune(lwbnd);
        }

        // Stopping criterion
        mxAssert( state->bnd >= lwbnd,  "Bound error");
        if (state->bnd - lwbnd <= gap)
        {
            delete state;
            break;
        }

        // Branch
        np = reg::search::split(state->ssr, ssr_array);
        delete state;

        for(i=0; i<np; i++)
        {
            dsi.contrast_bound(state->ssr, TRUE);
            upbnd = dsi.evalUpperBound(*(ssr_array[i]));
            if ( upbnd > lwbnd )
            {
                state = new SearchState<SSR>(*(ssr_array[i]), upbnd);
                queue.push(state);
            }
            delete ssr_array[i];
        }
        count_eval_ubbnd += np;
    }

    delete []ssr_array;
    return lwbnd;
}


template <class SSR, unsigned int BRANCHING_FACTOR>
int bnb_search_table(   DataIndexation<SSR> &dsi0,
                        DataIndexation<SSR> &dsi1,
                        DataIndexation<SSR> &dsi2,
                        DataIndexation<SSR> &dsi3,
                        DataIndexation<SSR> &dsi4,
                        DataIndexation<SSR> &dsi5,
                        DataIndexation<SSR> &dsi6,
                        DataIndexation<SSR> &dsi7,
                        int lwbnd, int gap, int buckets,
                        SSR &guessAndResult)
{
    int i, np;
    int upbndi;

    int count_eval_ubbnd;
    int count_eval_lwbnd;

    SearchState<SSR, int> *state;
    SSR **ssr_array; //array of pointers

    // Initialise
    count_eval_ubbnd = count_eval_lwbnd = 0;
    dsi0.contrast_bound(guessAndResult, TRUE);
    upbndi = dsi0.evalUpperBound(guessAndResult);
    count_eval_ubbnd++;


    if (upbndi-lwbnd <= gap)
    {
        return lwbnd;
    }

    StatePriorityHashtable<SSR, int, SearchState > table(buckets);


    ssr_array = new SSR*[BRANCHING_FACTOR];
    state = new SearchState<SSR>(guessAndResult, upbndi);
    table.push(state);

    int iter = 0;


    while (table.size())
    {
        iter++;

        // Find the state with the highest upper bound
        state = table.pop();

        // Evaluate lower boud
        /*dsi.contrast_bound(state->ssr, FALSE);
        state_lwbnd = dsi.evalLowerBound( state->ssr );
        count_eval_lwbnd++;

        // Update solution
        if (state_lwbnd > lwbnd)
        {
            lwbnd = state_lwbnd;
            guessAndResult = state->ssr;
            table.prune(lwbnd);
        }*/

        if(iter%15000==0)
        {
          break;
            //std::cout<< "bnb: "<< iter <<" lwbnd "<<lwbnd<<" upbnd "<<state->bnd
            //         <<" tablesize "<<table.size()<<std::endl;
        }

        // Stopping criterion
        mxAssert( state->bnd >= lwbnd, "Bound error");
        if (state->bnd - lwbnd <= gap)
        {
            delete state;
            break;
        }

        // Branch
        np = reg::search::split(state->ssr, ssr_array);
        int state_lwbnd[np];
        int upbnd[np];
        delete state;
        #pragma omp parallel sections
        {
          #pragma omp section
          {
            dsi0.contrast_bound(*(ssr_array[0]), TRUE);
            state_lwbnd[0] = dsi0.evalLowerBound(*(ssr_array[0]));
            upbnd[0] = dsi0.evalUpperBound(*(ssr_array[0]));

          }
          #pragma omp section
          {
            dsi1.contrast_bound(*(ssr_array[1]), TRUE);
            state_lwbnd[1] = dsi1.evalLowerBound(*(ssr_array[1]));
            upbnd[1] = dsi1.evalUpperBound(*(ssr_array[1]));
          }
          #pragma omp section
          {
            dsi2.contrast_bound(*(ssr_array[2]), TRUE);
            state_lwbnd[2] = dsi2.evalLowerBound(*(ssr_array[2]));
            upbnd[2] = dsi2.evalUpperBound(*(ssr_array[2]));
          }
          #pragma omp section
          {
            dsi3.contrast_bound(*(ssr_array[3]), TRUE);
            state_lwbnd[3] = dsi3.evalLowerBound(*(ssr_array[3]));
            upbnd[3] = dsi3.evalUpperBound(*(ssr_array[3]));
          }
          #pragma omp section
          {
            dsi4.contrast_bound(*(ssr_array[4]), TRUE);
            state_lwbnd[4] = dsi4.evalLowerBound(*(ssr_array[4]));
            upbnd[4] = dsi4.evalUpperBound(*(ssr_array[4]));
          }
          #pragma omp section
          {
            dsi5.contrast_bound(*(ssr_array[5]), TRUE);
            state_lwbnd[5] = dsi5.evalLowerBound(*(ssr_array[5]));
            upbnd[5] = dsi5.evalUpperBound(*(ssr_array[0]));
          }
          #pragma omp section
          {
            dsi6.contrast_bound(*(ssr_array[6]), TRUE);
            state_lwbnd[6] = dsi6.evalLowerBound(*(ssr_array[6]));
            upbnd[6] = dsi6.evalUpperBound(*(ssr_array[6]));
          }
          #pragma omp section
          {
            dsi7.contrast_bound(*(ssr_array[7]), TRUE);
            state_lwbnd[7] = dsi7.evalLowerBound(*(ssr_array[7]));
            upbnd[7] = dsi7.evalUpperBound(*(ssr_array[7]));
          }
        }

        for(i=0;i<np;i++)
        {
            //dsi.contrast_bound(*(ssr_array[i]), TRUE);
            //state_lwbnd[i] = dsi.evalLowerBound(*(ssr_array[i]));
            //upbnd[i] = dsi.evalUpperBound(*(ssr_array[i]));

            if ( upbnd[i] > lwbnd )
            {
                state = new SearchState<SSR>(*(ssr_array[i]), upbnd[i]);
                table.push(state);
            }
            if (state_lwbnd[i] > lwbnd)
            {
                lwbnd = state_lwbnd[i];
                guessAndResult = *(ssr_array[i]);
                table.prune(lwbnd);
            }
            delete ssr_array[i];
        }
        count_eval_ubbnd += np;
        count_eval_lwbnd += np;
    }
    cout<<iter<<endl;
    delete []ssr_array;
    return lwbnd;
}



    //TODO: indexation data structure should be in the template...
    template <class SSR, unsigned int BRANCHING_FACTOR>
    int searchTableDF(const DataIndexation<SSR> &dsi,
                         int lwbnd, int gap, int buckets,
                         SSR &guessAndResult)
    {
        int i, np;
        int state_lwbnd, upbnd;

        int count_eval_ubbnd;
        int count_eval_lwbnd;

        SearchState<SSR, int> *state;
        SSR **ssr_array; //array of pointers

        // Initialise
        count_eval_ubbnd = count_eval_lwbnd = 0;


        upbnd = dsi.evalUpperBound(guessAndResult, lwbnd);

        count_eval_ubbnd++;


        if (upbnd-lwbnd <= gap)
        {
            return lwbnd;
        }

        StatePriorityHashtableDF<SSR, int, SearchState > table(buckets);


        ssr_array = new SSR*[BRANCHING_FACTOR];
        state = new SearchState<SSR>(guessAndResult, upbnd);
        table.push(state);

        int iter = 0;

        while (table.size())
        {
            iter++;

            // Find the state with the highest upper bound
            state = table.pop();

            // Evaluate lower boud
            state_lwbnd = dsi.evalLowerBound( state->ssr );
            count_eval_lwbnd++;

            // Update solution
            if (state_lwbnd > lwbnd)
            {
                lwbnd = state_lwbnd;
                guessAndResult = state->ssr;
                table.prune(lwbnd);
            }

            if(iter%1000000==0)
            {
                std::cout<< "bnb: "<< iter <<" lwbnd "<<lwbnd<<" upbnd "<<state->bnd
                <<" tablesize "<<table.size()<<std::endl;
            }

            // Stopping criterion
            mxAssert( state->bnd >= lwbnd, "Bound error");
            if (state->bnd - lwbnd <= gap)
            {
                delete state;
                break;
            }

            // Branch
            np = reg::search::split(state->ssr, ssr_array);
            delete state;

            for(i=0; i<np; i++)
            {
                upbnd = dsi.evalUpperBound(*(ssr_array[i]), lwbnd);
                if ( upbnd > lwbnd )
                {
                    state = new SearchState<SSR>(*(ssr_array[i]), upbnd);
                    table.push(state);
                }
                delete ssr_array[i];
            }
            count_eval_ubbnd += np;
        }

        delete []ssr_array;
        return lwbnd;
    }




// Search using Matching Lists
template <class SSR, unsigned int BRANCHING_FACTOR>
int bnb_search_ml_table(const DataIndexation<SSR> &dsi,
                         int lwbnd, int gap, int buckets,
                         SSR &guessAndResult)
{
    int i,j,k, np;
    int state_lwbnd, upbnd;

    int count_eval_ubbnd;
    int count_eval_lwbnd;
    int *matchList;
    const size_t dsiSize = dsi.size();

    SearchStateML<SSR> *state, *childState;
    SSR **ssr_array; //array of pointers

    // Initialise
    count_eval_ubbnd = count_eval_lwbnd = 0;

    matchList = new int[dsiSize];
    for(i=0; i<dsiSize; i++)
    {
        matchList[i]=i;
    }

    std::vector<bool> matchesMap(dsiSize);
    upbnd = dsi.evalUpperBound(guessAndResult, lwbnd, matchList, dsiSize, matchesMap);
    count_eval_ubbnd++;

    if (upbnd-lwbnd <= gap)
    {
        delete matchList;
        return lwbnd;
    }

    mxAssert(upbnd == dsi.evalUpperBound(guessAndResult,lwbnd), "");

    StatePriorityHashtable<SSR, int, SearchStateML > table(buckets);


    //Create new matchlist
    i=0;
    for(j=0; j<dsiSize; j++)
    {
        if(matchesMap[j])
        {
            matchList[i++]=matchList[j];
        }
    }
    mxAssert(i<=upbnd,"" );

    ssr_array = new SSR*[BRANCHING_FACTOR];

    state = new SearchStateML<SSR>(guessAndResult, upbnd, matchList, i);
    table.push(state);

    int iter = 0;
    while (table.size())
    {
        iter++;

        // Find the state with the highest upper bound
        state = table.pop();

        // Evaluate lower boud
        state_lwbnd = dsi.evalLowerBound(state->ssr, state->matchList, state->matchListSize);
        count_eval_lwbnd++;

        // Update solution
        if (state_lwbnd > lwbnd)
        {
            lwbnd = state_lwbnd;
            guessAndResult = state->ssr;
            table.prune(lwbnd);
        }

        if(iter%100000==0)
        {
            std::cout<< "bnb ml: "<< iter <<" lwbnd "<<lwbnd<<" upbnd "<<state->bnd
                     <<" table size "<<table.size()<<" state "<<state->ssr<<std::endl;
        }

        // Stopping criterion
        mxAssert( state->bnd >= lwbnd, "Bound error");
        if (state->bnd - lwbnd <= gap)
        {
            delete state;
            break;
        }

        // Branch
        np = reg::search::split(state->ssr, ssr_array);
        for(i=0; i<np; i++)
        {
            upbnd = dsi.evalUpperBound(
                        *(ssr_array[i]), lwbnd,
                        state->matchList, state->matchListSize, matchesMap);

            if ( upbnd > lwbnd )
            {
                mxAssert(upbnd==dsi.evalUpperBound(*(ssr_array[i]), lwbnd), "inconsistent result");


                matchList = new int[upbnd];
                k=0;
                for(j=0; j<state->matchListSize; j++)
                {
                    if(matchesMap[j])
                    {
                        matchList[k++]=state->matchList[j];
                    }
                }
                mxAssert(k<=upbnd,"error building matchList");

                childState = new SearchStateML<SSR>(*(ssr_array[i]), upbnd, matchList, k);
                table.push(childState);
            }
            delete ssr_array[i];
        }
        delete state;
        count_eval_ubbnd += np;
    }
    delete []ssr_array;

    return lwbnd;
}

} // End namespace reg
} // End namespace search

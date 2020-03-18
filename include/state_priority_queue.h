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

#ifndef REG_STATE_PRIORITY_QUEUE_H_
#define REG_STATE_PRIORITY_QUEUE_H_

#include "state.h"
#include <cstddef> //NULL

namespace reg {
namespace search {


template <class SSR, typename Scalar=int >
class StatePriorityQueue
{
public:
    enum OptProblem{MINIMISATION, MAXIMISATION};

private:

    class Node
    {
    public:
        SearchState<SSR, Scalar> *state;
        Node *left, *right;

        Node(): state(NULL), left(NULL), right(NULL) {}
        Node(SearchState<SSR, Scalar> *state):state(state), left(NULL), right(NULL){}
        ~Node() {if (state!=NULL) delete state;}
    };

    const OptProblem optProblem;
    Node *head, *tail;
    unsigned int m_size;

public:
    StatePriorityQueue(OptProblem op=MAXIMISATION);
    ~StatePriorityQueue();

    SearchState<SSR, Scalar> *pop();
    void push(SearchState<SSR, Scalar> *state);

    /**
     * @brief Remove and free states with upper bound lower or equal to lwbnd.
     * @param lwbnd Known lower bound.
     */
    void prune(Scalar curbest);

    unsigned int size() const;
};


} // End namespace search
} // End namespace reg

#include "state_priority_queue.hpp"

#endif

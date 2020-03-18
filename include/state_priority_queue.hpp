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
#include <time.h>
#include <stdlib.h>
#include <cstdio>

namespace reg
{
namespace search
{


template <class SSR, typename Scalar>
StatePriorityQueue<SSR, Scalar>::StatePriorityQueue(OptProblem op):
    optProblem(op),
    head(NULL), tail(NULL), m_size(0) {}

template <class SSR, typename Scalar>
StatePriorityQueue<SSR, Scalar>::~StatePriorityQueue()
{
    Node *node, *tmp;

    node = head;
    while(node)
    {
       tmp = node;
       node = node->right;
       delete tmp;
    }
}


template <class SSR, typename Scalar>
SearchState<SSR, Scalar> *StatePriorityQueue<SSR, Scalar>::pop()
{
    SearchState<SSR, Scalar> *res;

    if( head==NULL )
    {
        return NULL;
    }

    Node *tmp=head;
    res = head->state;

    head = head->right;
    if (head != NULL)
    {
        head->left = NULL;
    }
    else
    {
        tail = NULL;
    }

    tmp->state=NULL;
    delete tmp;
    m_size--;
    return res;
}


template <class SSR, typename Scalar>
void StatePriorityQueue<SSR, Scalar>::push(SearchState<SSR, Scalar> *state)
{
    Node *node, *new_node;
    new_node = new Node(state);

    if (m_size==0)
    {
        head = tail = new_node;
        m_size = 1;
        return;
    }

    // Traverse queue from tail to insert node in descent order
    node = tail;
    while (node)
    {
        if ((optProblem==MAXIMISATION && node->state->bnd >= state->bnd) ||
            (optProblem==MINIMISATION && node->state->bnd <= state->bnd) )
        {
            // Inserting at the end of the queue
            if (node==tail)
            {
                tail = new_node;
            }
            // Inserting node between head and tail nodes
            else
            {
                new_node->right = node->right;
                new_node->right->left = new_node;
            }

            // Link new_node to the left node
            new_node->left = node;
            node->right = new_node;

            m_size++;
            return;
        }
        node = node->left;
    }

    // Inserting at the head
    new_node->right = head;
    new_node->right->left = new_node;
    head = new_node;
    m_size++;
}

template <class SSR, typename Scalar>
void StatePriorityQueue<SSR, Scalar>::prune(Scalar curbest)
{
    Node *node, *tmp_node;

    // Traverse queue from tail to delete nodes
    node = tail;
    while (node)
    {
        // Stop condition
        if ( (optProblem==MAXIMISATION && node->state->bnd > curbest) ||
             (optProblem==MINIMISATION && node->state->bnd < curbest))
        {
            tail = node;
            tail->right = NULL;
            return;
        }

        // Delete visited node
        m_size--;
        tmp_node = node;
        node = node->left;
        delete tmp_node;
    }

    // Boundary case: All nodes were deleted
    head = tail = NULL;
    mxAssert(m_size>=0, "");
}

template <class SSR, typename Scalar>
unsigned int StatePriorityQueue<SSR, Scalar>::size() const
{
    mxAssert(m_size>=0,"");
    return m_size;
}


} // End namespace search
} // End namespace reg


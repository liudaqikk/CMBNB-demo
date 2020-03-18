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

#ifndef REG_BINARYTREE_H
#define REG_BINARYTREE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "math.h"


namespace reg
{
namespace binarytree {


/* Definition for interval.*/
typedef struct interval
{
    double lw;
    double up;
} interval;

/* Definitions for binary search tree.*/
typedef struct payload
{
    double val;
    int order;
} payload;

typedef struct treeNode
{
    payload data;
    struct treeNode *left;
    struct treeNode *right;
} treeNode;

treeNode *Insert(treeNode*, payload, treeNode*);
void free_Binarytree(treeNode *node);
int queryLower(treeNode*,double,treeNode*);
int queryUpper(treeNode*,double,treeNode*);
double queryMiddle(treeNode *,double,treeNode *);
void PrintInorder(treeNode*);
int size_Binarytree(treeNode*);
int count_pointers_Binarytree(treeNode*);

} // End namespace binarytree
} // End namespace reg

#endif

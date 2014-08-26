#ifndef UTILS_HH
#define UTILS_HH


/**
 *  @brief utils.hpp defines the common utilities used by the project
 *
 *  Currently defines the union find structure for the segmentation. The
 *  functions are implemented in C in order to use them in CUDA too.
 */


/**
 *  Structure for the elements on the Union-Find data structure.
 */
struct UnionFindElem
{
    int parent;
    int rank;
};


/**
 *  @brief find the subset to which the element 'i' belongs.
 *
 *  @param elements The array of elements.
 *  @param i The index of the element of interest.
 *
 *  @return The resulting set to which the element i belongs
 */
int find(UnionFindElem *elements, int i);


/**
 *  @brief Union function to join two sets on the data structure
 *
 *  @param elements The set of elements in the data structure
 *  @param x The element in the first set
 *  @param y The element in the second set
 */
void union_join(UnionFindElem *elements, int x, int y);

#endif

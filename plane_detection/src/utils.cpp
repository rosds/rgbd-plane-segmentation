#include <utils.hpp>

int find(UnionFindElem *elements, int i)
{
    if (elements[i].parent != i) {
        elements[i].parent = find(elements, elements[i].parent);
    }
    return elements[i].parent;
}

void union_join(UnionFindElem *elements, int x, int y)
{
    int xset = find(elements, x);
    int yset = find(elements, y);

    if (elements[xset].rank < elements[yset].rank) {
        elements[xset].parent = yset;
    }
    else if (elements[xset].rank > elements[yset].rank) {
        elements[yset].parent = xset;
    }
    else {
        elements[yset].parent = xset;
        elements[xset].rank++;
    }
}

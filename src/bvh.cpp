#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>
#include <limits.h>
#include <assert.h>

using namespace std;

namespace CMU462 { namespace StaticScene {

typedef std::vector<Primitive *>::iterator PrimitiveIter;
typedef std::vector<BBox *>::iterator BBoxIter;
typedef std::vector<int>::iterator IntIter;

double splitCost(BBoxIter begin, BBoxIter end, BBoxIter split,
                 int numLeft, int numRight)
{
    BBox left, right, total;
    for (BBoxIter ii = begin; ii < split; ii++) {
        total.expand(**ii);
        left.expand(**ii);
    }
    for (BBoxIter ii = split; ii < end; ii++) {
        total.expand(**ii);
        right.expand(**ii);
    }
    double leftCost = left.surface_area() / total.surface_area() *
                      numLeft;
    double rightCost = right.surface_area() / total.surface_area() *
                      numRight;
    return leftCost + rightCost;
}

// Recursively build a BVH of the given vector
BVHNode *buildBvh(PrimitiveIter begin, int beginIndex,
                 PrimitiveIter end, size_t max_leaf_size)
{
    BBox bb;
    Vector3D max, min, extent;
    const int numBuckets = 16;
    double minCost = numeric_limits<double>::max();
    double minDim;
    double minSplit;
    PrimitiveIter splitPrim;
    int numElems = 0;

    // Termination condition if we have < 4 elements
    // Make the bb for this section of the BVH
    for (PrimitiveIter ii = begin; ii < end; ++ii) {
        bb.expand((*ii)->get_bbox());
        numElems++;
    }

    // If we have less than our leaf size, terminate now
    if (numElems <= max_leaf_size) {
        return new BVHNode(bb, beginIndex, numElems);
    }

    max = bb.max;
    min = bb.min;
    extent = bb.extent;

    // Initialize the min cost to be based off no split
    minCost = numElems;
    minDim = 0;
    minSplit = min[0];

    // Try splitting planes across all dimensions
    for (int dim = 0; dim < 3; dim++) {
        // Initialize our buckets
        std::vector<BBox *> buckets;
        int bucketSizes[numBuckets];

        if (extent[dim] == 0) {
            continue;
        }
        for (int ii = 0; ii < numBuckets; ii++) {
            buckets.push_back(new BBox());
            bucketSizes[ii] = 0;
        }

        // Iterate over elements and add to buckets
        for (PrimitiveIter pi = begin; pi < end; pi++) {
            // Normalize centroid to be [0,1] on axis extent
            double loc = ((*pi)->get_bbox().centroid()[dim] - min[dim]) /
                         extent[dim];

            // Multiply by num buckets to get the bucket
            int bucket = loc * numBuckets;
            buckets[bucket]->expand((*pi)->get_bbox());
            bucketSizes[bucket]++;
        }

        // Iterate over plane splits to find the optimal plane split for
        // this dimension of buckets. Note that there are B-1 potential
        // splits for B buckets
        for (int ii = 0; ii < numBuckets-1; ii++){
            int numLeft = 0;
            int numRight = 0;

            for (int jj = 0; jj < ii+1; jj++) {
                numLeft += bucketSizes[jj];
            }
            for (int jj = ii+1; jj < numBuckets; jj++) {
                numRight += bucketSizes[jj];
            }
            // If this isn't a real split, don't consider it
            if (numLeft == 0 || numRight == 0) {
                continue;
            }
            BBoxIter rightHalfBegin = buckets.begin() + (ii+1);
            double cost = splitCost(buckets.begin(), buckets.end(),
                                    rightHalfBegin, numLeft, numRight);
            if (cost < minCost) {
                double progress = (double)(ii+1) / numBuckets;
                minCost = cost;
                minDim = dim;
                minSplit = min[dim] + progress * extent[dim];
            }
        }
    }

    // Split based off the minimum split found
    splitPrim = std::partition(begin, end,
                    [minSplit, minDim](Primitive *p){
                        return p->get_bbox().centroid()[minDim] <
                        minSplit;});

    size_t leftElems = splitPrim - begin;
    size_t rightStart = beginIndex + leftElems;
    size_t rightElems = numElems - leftElems;
    if ((leftElems == 0) || (rightElems == 0)) {
        return new BVHNode(bb, beginIndex, numElems);
    }
    assert(leftElems != 0);
    assert(rightElems != 0);
    // We now have the optimal splitting plane of the 3N considered
    // Partition based on this plane and recurse
    BVHNode *left = buildBvh(begin, beginIndex, splitPrim, max_leaf_size);
    BVHNode *right = buildBvh(splitPrim, rightStart, end, max_leaf_size);

    BVHNode *myNode = new BVHNode(bb, beginIndex, numElems);
    myNode->l = left;
    myNode->r = right;
    return myNode;
}

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
    this->primitives = _primitives;

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code build a BVH aggregate with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.


    root = buildBvh(primitives.begin(), 0, primitives.end(), max_leaf_size);
}

void deleteNode(BVHNode *n) {
    if (n->isLeaf()) {
        return;
    }
    deleteNode(n->l);
    deleteNode(n->r);
    delete n;
}

BVHAccel::~BVHAccel() {
    // Implement a proper destructor for your BVH accelerator aggregate
    deleteNode(root);
    root = NULL;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

bool find_closest_hit(const std::vector<Primitive *> &primitives,
                      const Ray &ray, BVHNode *node, Intersection *i) {
    double this0, this1;
    bool thisHit;
    bool hit = false;
    thisHit = node->bb.intersect(ray, this0, this1);
    if (!thisHit || (this0 > ray.max_t)) {
        return false;
    }

    if (node->isLeaf()) {
        for (int ii = node->start; ii < node->start+node->range; ii++) {
            // this will update the intersection structure if there is a hit
            if (i != NULL) {
                hit = primitives[ii]->intersect(ray, i) ? true : hit;
            } else {
                hit = primitives[ii]->intersect(ray) ? true : hit;
            }
        }
    } else {
        double l0, l1, r0, r1;
        bool lhit, rhit;
        
        lhit = node->l->bb.intersect(ray, l0, l1);
        rhit = node->r->bb.intersect(ray, r0, r1);

        if (!lhit && !rhit) {
            return false;
        }
        BVHNode *first = (l0 < r0) ? node->l : node->r;
        bool fhit = (l0 < r0) ? lhit : rhit;
        BVHNode *second = (l0 < r0) ? node->r : node->l;
        bool shit = (l0 < r0) ? rhit : lhit;

        if (fhit) {
            fhit = find_closest_hit(primitives, ray, first, i);
        }
        if (shit) {
            shit = find_closest_hit(primitives, ray, second, i);
        }
        hit = fhit || shit;
    }
    return hit;
}

bool BVHAccel::intersect(const Ray &ray) const {
    // Implement ray - bvh aggregate intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.
    bool hit = find_closest_hit(primitives, ray, root, NULL);
    return hit;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {
    // Implement ray - bvh aggregate intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate. When an intersection does happen.
    // You should store the non-aggregate primitive in the intersection data
    // and not the BVH aggregate itself.

    bool hit = find_closest_hit(primitives, ray, root, i);
    return hit;
}

}  // namespace StaticScene
}  // namespace CMU462

#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuildSVH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

float calS(Vector3f diag) {
    return (diag.x * diag.y + diag.y * diag.z + diag.x * diag.z) * 2;
}

BVHBuildNode* BVHAccel::recursiveBuildSVH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    const int B = 100;

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildSVH(std::vector{objects[0]});
        node->right = recursiveBuildSVH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        float min_c = std::numeric_limits<float>::max();
        int min_leftn = 0;
        int leftn = 0;
        float left, intervel = 0;
        int index = 0;
        // std::cout << dim << '\n';
        switch (dim) {
        case 0:
            left = centroidBounds.pMin.x;
            intervel = (centroidBounds.pMax.x - centroidBounds.pMin.x) / B * 1.0f;
            // std::cout << centroidBounds.pMin << " ~ " << centroidBounds.pMax << " :\t" << intervel << '\n';
            break;
        case 1:
            left = centroidBounds.pMin.y;
            intervel = (centroidBounds.pMax.y - centroidBounds.pMin.y) / B * 1.0f;
            // std::cout << centroidBounds.pMin << " ~ " << centroidBounds.pMax << " :\t" << intervel << '\n';
            break;
        case 2:
            left = centroidBounds.pMin.z;
            intervel = (centroidBounds.pMax.z - centroidBounds.pMin.z) / B * 1.0f;
            // std::cout << centroidBounds.pMin << " ~ " << centroidBounds.pMax << " :\t" << intervel << '\n';
            break;
        default:
            // std::cout << "default branch";
            break;
        }
        float middle = left;
        for (int i = 0; i < objects.size();) {
            bool update = false;
            auto obj = objects[i];
            switch (dim) {
            case 0:
                if (obj->getBounds().Centroid().x > middle) {
                    update = true;
                } else {
                    i++;
                }
                break;
            case 1:
                if (obj->getBounds().Centroid().y > middle) {
                    update = true;
                } else {
                    i++;
                }
                break;
            case 2:
                if (obj->getBounds().Centroid().z > middle) {
                    update = true;
                } else {
                    i++;
                }
                break;
            }
            if (update) {
                Vector3f diag = centroidBounds.Diagonal();
                float s_left, s_right;
                switch (dim) {
                case 0:
                    s_left = calS(Vector3f(diag.x * index / B, diag.y, diag.z));
                    s_right = calS(Vector3f(diag.x * (1 - 1.0f * index / B), diag.y, diag.z));
                    break;
                case 1:
                    s_left = calS(Vector3f(diag.x, diag.y * index / B, diag.z));
                    s_right = calS(Vector3f(diag.x, diag.y * (1 - 1.0f * index / B), diag.z));
                    break;
                case 2:
                    s_left = calS(Vector3f(diag.x, diag.y, diag.z * index / B));
                    s_right = calS(Vector3f(diag.x, diag.y, diag.z * (1 - 1.0f * index / B)));
                    break;
                }

                float cur_c = (i * s_left + (objects.size() - i) * s_right) / centroidBounds.SurfaceArea();
                if (cur_c < min_c) {
                    min_c = cur_c;
                    min_leftn = i;
                }

                middle += intervel;
                update = false;

                index++;
            }
        }
        // std::cout << "recurse start\n" << min_leftn << " : " << objects.size() << '\n';
        middling = beginning + min_leftn;
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        // std::cout << "assert started\n";
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        // std::cout << "assert passed\n";
        node->left = recursiveBuildSVH(leftshapes);
        node->right = recursiveBuildSVH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        // std::cout << "recurse finished\n";
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if (!node->bounds.IntersectP(ray, ray.direction_inv, std::array<int, 3>{ {0, 0, 0} }))
        return Intersection();
    
    if (node->left && node->right) {
        Intersection left_intersection = getIntersection(node->left, ray);
        Intersection right_intersection = getIntersection(node->right, ray);
        if (left_intersection.happened || right_intersection.happened) {
            return left_intersection.distance < right_intersection.distance ? left_intersection : right_intersection;
        }
        return Intersection();
    }

    return node->object->getIntersection(ray);

}
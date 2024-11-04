




#include "owoaccel.h"

namespace pbrt{    
    
    Bounds3f OwOAccel::WorldBound() const
    {
        return bounds;
    }

    OwOAccel::~OwOAccel()
    {
    }

    OwOAccel::OwOAccel(std::vector<std::shared_ptr<Primitive>> p)
    {
        primitives = p;

        // Compute bounds for kd-tree construction
        std::vector<Bounds3f> primBounds;
        primBounds.reserve(primitives.size());
        for (const std::shared_ptr<Primitive>& prim : primitives)
        {
            Bounds3f b = prim->WorldBound();
            bounds = Union(bounds, b);
            primBounds.push_back(b);
        }

        //Create BoundingBox
        float minX = std::min(bounds.Corner(0).x, bounds.Corner(1).x);
        float maxX = std::max(bounds.Corner(0).x, bounds.Corner(1).x);

        float minY = std::min(bounds.Corner(0).y, bounds.Corner(1).y);
        float maxY = std::max(bounds.Corner(0).y, bounds.Corner(1).y);

        float minZ = std::min(bounds.Corner(0).z, bounds.Corner(1).z);
        float maxZ = std::max(bounds.Corner(0).z, bounds.Corner(1).z);

        BoundingBox boundingBox = BoundingBox(minX, maxX, minY, maxY, minZ, maxZ);

        //Create Octree
        root = OctreeSegment(boundingBox, &primitives);
        root.SplitOctree();
    }

    bool OwOAccel::Intersect(const Ray& ray, SurfaceInteraction* isect) const
    {
        //// No primitives to intersect with
        //if (primitives.empty())
        //{
        //    return false;
        //}

        //int prevSect = -1;
        //float isectPrevDepth = std::numeric_limits<float>::infinity();
        //SurfaceInteraction isectTemp;

        //for (int i = 0; i < primitives.size(); i++)
        //{
        //    // Run hit test on single primitive
        //    if (primitives[i]->Intersect(ray, &isectTemp))
        //    {
        //        float newDepth = CalculateIntersectDepth(isectTemp.p, ray.o);
        //        if (prevSect == -1 || newDepth < isectPrevDepth)
        //        {
        //            isectPrevDepth = newDepth;
        //            prevSect = i;

        //            // Store the closest intersection details
        //            *isect = isectTemp; 
        //        }
        //    }

        //}

        ////Check if a hit occured
        //if (prevSect == -1)
        //{
        //    return false;
        //}

        ////Determine which is the closest hit
        //return true;


        // ================================== Octree Version ==================================
        return root.Intersect(ray, isect);
        // ================================== Octree Version ==================================
    }

    float OwOAccel::CalculateIntersectDepth(Point3f isectPoint, Point3f rayOrigin) const
    {
        return (isectPoint - rayOrigin).LengthSquared();
    }

    bool OwOAccel::IntersectP(const Ray& ray) const
    {
        // No primitives to intersect with
        if (primitives.empty())
        {
            return false;
        }

        for (int i = 0; i < primitives.size(); i++)
        {
            // Run hit test on single primitive
            if (primitives[i]->IntersectP(ray))
            {
                return true;
            }

        }

        return false;
    }

    std::shared_ptr<OwOAccel> CreateOwOAccelerator(std::vector<std::shared_ptr<Primitive>> prims, const ParamSet& ps) 
    {

        return std::make_shared<OwOAccel>(std::move(prims));
    }





    //======================================== Octree Segment Part ========================================

    OwOAccel::OctreeSegment::OctreeSegment(BoundingBox bounds, std::vector<std::shared_ptr<Primitive>>* realPrimitives)
    {
        //Assign Bounds
        this->bounds = bounds;
        pbrtBounds = Bounds3f(Point3f(bounds.minX, bounds.minY, bounds.minZ), Point3f(bounds.maxX, bounds.maxY, bounds.maxZ));

        //Assign the real primitives
        this->realPrimitives = realPrimitives;
    }

    //Intersect test trough Octree
    // => First we test if the ray even hits the bounds of the octree segment
    // => Then we check if the ray hits each segment, if the octree does have segments, and how close those hits are to the ray
    // => Then we check which hit was the closest, and do the intersect tests for that segment
    // => Once we are a leaf node, we do intersection tests for all prims in the leaf and return the closest to the ray
    // => If a segment does not have intersections, we check the next closest segment, if there is one
    // => Once we have the intersect point of the segment, i.e the closest segment, we do the intersection tests for all orphan primitives
    // => Then we compare the orphan primitives check to the results from our nodes and pick the closest one
    bool OwOAccel::OctreeSegment::Intersect(const Ray& ray, SurfaceInteraction* isect) const
    {
        //First check if ray even hits the bounds of octree segment
        if (pbrtBounds.IntersectP(ray))
        {
            //If it does hit, check each segment, if they exist, for hits
            // => Save the hit children
            //After this, we need to see which children is the closest to the ray origin and traverse that first
            std::vector<int> hitChildren = std::vector<int>();
            SurfaceInteraction* childrenInteraction;
            if (childSegments != nullptr)
            {
                //Go trough all children
                for (int i = 0; i < 8; i++)
                {
                    if (childSegments[i]->pbrtBounds.IntersectP(ray))
                    {
                        hitChildren.push_back(i);
                    }
                }


                //Go trough the closest hit children as long as we do not find a valid Intersection
                bool validIntersectionFound = false;
                while (!validIntersectionFound && !hitChildren.empty())
                {
                    //Get intersection from new closest child
                    int closestChild = hitChildren.at(0);
                    float closestChildDist = std::numeric_limits<float>::infinity();
                    for (int i = 0; i < hitChildren.size(); i++)
                    {
                        float dist = (childSegments[i]->bounds.GetMiddle() - ray.o).LengthSquared();
                        if (dist < closestChildDist)
                        {
                            closestChildDist = dist;
                            closestChild = i;
                        }
                    }

                    //Make intersection test with child
                    if (childSegments[closestChild]->Intersect(ray, childrenInteraction))
                    {
                        validIntersectionFound = true;
                    }

                    //If the closest child does not have any intersections
                    // => Remove it from the hitChildren and continue
                    else
                    {
                        hitChildren.erase(std::remove(hitChildren.begin(), hitChildren.end(), closestChild), hitChildren.end());
                    }
                }

            }

            //If we do not have children, i.e we are a leaf node
            // => Do intersection test for all of our primitives and determine which is the closest one
            // => Once closest one found, set that as isect and return with true, assuming a hit was found
            else
            {
                return IntersectVector(ray, isect, primitives);
            }

            //We can be here, if we do have children and got their closest hit
            // => Hit our orphans, and compare to the child hit
            // => Whichever is the closest gets  set as the isect and returned

            //First try and hit all orphan primitives
            //if there is an orphan hit, compare them
            SurfaceInteraction* orphanHit;
            if (IntersectVector(ray, orphanHit, orphanPrimitives))
            {
                if (childrenInteraction != nullptr)
                {
                    if ((orphanHit->p - ray.o).LengthSquared() < (childrenInteraction->p - ray.o).LengthSquared())
                    {
                        isect = orphanHit;
                        return true;
                    }
                }

                else
                {
                    isect = orphanHit;
                    return true;
                }

            }

            //No orphan hit => Return the intersection of children (if it exists)
            if (childrenInteraction != nullptr)
            {
                isect = childrenInteraction;
                return true;
            }
        }

        return false;
    }

    //Function that splits this octree segment if it has more than the max amount of primitives
    //To do so we need to:
    // => Create 4 new segments of Octree
    // => For each Segment check if a primitive is inside
    void OwOAccel::OctreeSegment::SplitOctree()
    {
        //Create 4 new segments with half bounds
        childSegments = new OctreeSegment*[8];

        //Octree midPoints
        float midX = (bounds.minX + bounds.maxX) / 2;
        float midY = (bounds.minY + bounds.maxY) / 2;
        float midZ = (bounds.minZ + bounds.maxZ) / 2;

        //Split Octree an Create BoundingBoxes
        for (int i = 0; i < 8; i++)
        {
            bool isLeft = (i & 1) == 0;
            bool isBottom = (i & 2) == 0;
            bool isBack = (i & 4) == 0;

            float minX = isLeft ? bounds.minX : midX;
            float maxX = isLeft ? midX : bounds.maxX;
            float minY = isBottom ? bounds.minY : midY;
            float maxY = isBottom ? midY : bounds.maxY;
            float minZ = isBack ? bounds.minZ : midZ;
            float maxZ = isBack ? midZ : bounds.maxZ;

            childSegments[i] = new OctreeSegment(BoundingBox(minX, maxX, minY, maxY, minZ, maxZ), realPrimitives);
        }

        //Go trough each octree segment and then go trough all primitives and check if they are inside the octree segments
        for (int i = 0; i < 8; i++)
        {
            for (int k = 0; k < primitives.size(); k++)
            {
                //If the primitive is inside the child segment
                if (childSegments[i]->IsInsideBounds(realPrimitives->at(primitives[i])->WorldBound()))
                {
                    //Add it to the child segment
                    childSegments[i]->primitives.push_back(k);
                }
            }
        }

        //After we have assigned all the primitives check for orphan primitives
        for (int prim : primitives)
        {
            if (std::find(assignedPrimitives.begin(), assignedPrimitives.end(), prim) == assignedPrimitives.end())
            {
                orphanPrimitives.push_back(prim);
            }
        }

        //Split each segment again if their primitives size is too big
        for (int i = 0; i < 8; i++)
        {
            if (childSegments[i]->primitives.size() > maxPrimsPerSegment)
            {
                childSegments[i]->SplitOctree();
            }
        }
    }

    bool OwOAccel::OctreeSegment::IsInsideBounds(Bounds3f primBounds)
    {
        for (int i = 0; i < 8; i++)
        {
            if (!bounds.Contains(primBounds.Corner(i)))
            {
                return false;
            }
        }

        return true;
    }

    bool OwOAccel::OctreeSegment::IntersectVector(const Ray& ray, SurfaceInteraction* isect, std::vector<int> primList) const
    {
        // No primitives to intersect with
        if (primList.empty())
        {
            return false;
        }

        int prevSect = -1;
        float isectPrevDepth = std::numeric_limits<float>::infinity();
        SurfaceInteraction isectTemp;

        for (int i = 0; i < primList.size(); i++)
        {
            // Run hit test on single primitive
            if (realPrimitives->at(primList[i])->Intersect(ray, &isectTemp))
            {
                float newDepth = (isectTemp.p - ray.o).LengthSquared();
                if (prevSect == -1 || newDepth < isectPrevDepth)
                {
                    isectPrevDepth = newDepth;
                    prevSect = i;

                    // Store the closest intersection details
                    *isect = isectTemp;
                }
            }

        }

        //Check if a hit occured
        if (prevSect == -1)
        {
            return false;
        }

        //Determine which is the closest hit
        return true;
    }

    bool OwOAccel::BoundingBox::Contains(Point3f point)
    {
        return (point.x >= minY && point.x <= maxX &&
            point.y >= minY && point.y <= maxY &&
            point.z >= minZ && point.z <= maxZ);
    }

    Point3f OwOAccel::BoundingBox::GetMiddle()
    {
        float midX = (minX + maxX) / 2;
        float midY = (minY + maxY) / 2;
        float midZ = (minZ + maxZ) / 2;
        return Point3f(midX, midY, midZ);
    }

    OwOAccel::BoundingBox::BoundingBox(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
    {
        this->minX = minX;
        this->maxX = maxX;
        this->minY = minY;
        this->maxY = maxY;
        this->minZ = minZ;
        this->maxZ = maxZ;
    }

    OwOAccel::BoundingBox::BoundingBox()
    {
    }

}
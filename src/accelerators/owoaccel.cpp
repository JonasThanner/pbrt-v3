




#include "owoaccel.h"

namespace pbrt{    
    
    Bounds3f OwOAccel::WorldBound() const
    {
        return Bounds3f();
    }

    OwOAccel::~OwOAccel()
    {
    }

    OwOAccel::OwOAccel(std::vector<std::shared_ptr<Primitive>> p)
    {
        primitives = p;
    }

    bool OwOAccel::Intersect(const Ray& ray, SurfaceInteraction* isect) const
    {
        // No primitives to intersect with
        if (primitives.empty())
        {
            return false;
        }

        int prevSect = -1;
        float isectPrevDepth = std::numeric_limits<float>::infinity();
        SurfaceInteraction isectTemp;
        for (int i = 0; i < primitives.size(); i++)
        {
            // Run hit test on single primitive
            if (primitives[i]->Intersect(ray, &isectTemp))
            {
                float newDepth = CalculateIntersectDepth(isectTemp.p, ray.o);
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

    float OwOAccel::CalculateIntersectDepth(Point3f isectPoint, Point3f rayOrigin) const
    {
        return (isectPoint - rayOrigin).Length();
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

}
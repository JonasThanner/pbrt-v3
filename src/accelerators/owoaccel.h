

#include "pbrt.h"
#include "primitive.h"

namespace pbrt {

class OwOAccel : public Aggregate
{
	public:
		    // BVHAccel Public Methods
          OwOAccel(std::vector<std::shared_ptr<Primitive>> p);
          Bounds3f WorldBound() const;
          ~OwOAccel();
          bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;
          bool IntersectP(const Ray &ray) const;



    private:        
        float CalculateIntersectDepth(Point3f isectPoint, Point3f rayOrigin) const;
        std::vector<std::shared_ptr<Primitive>> primitives;



};

std::shared_ptr<OwOAccel> CreateOwOAccelerator(std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}
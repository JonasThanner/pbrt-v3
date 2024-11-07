

#include "pbrt.h"
#include "primitive.h"

namespace pbrt {

class OwOAccel : public Aggregate
{
    class BoundingBox
    {
    public:
        float minX, maxX, minY, maxY, minZ, maxZ;
        bool Contains(Point3f point);
        Point3f GetMiddle();

    public:
        BoundingBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
        BoundingBox(Bounds3f pbrtBox);
        BoundingBox();
    };

    class OctreeSegment
    {
    public:
        OctreeSegment();
        OctreeSegment(BoundingBox bounds, std::vector<std::shared_ptr<Primitive>>* realPrimitives);
        Bounds3f pbrtBounds;
        BoundingBox bounds;
        int maxPrimsPerSegment = 10;
        bool Intersect(const Ray& ray, SurfaceInteraction* isect) const;
        void SplitOctree(int depth = 0);
        void AddPrimitive(int addition)
        {
            primitives.push_back(addition);
        };

    private:
        OctreeSegment** childSegments = nullptr;
        std::vector<int> primitives = std::vector<int>();
        std::vector<int> assignedPrimitives = std::vector<int>();
        std::vector<int> orphanPrimitives = std::vector<int>(); //Prims that didnt fit into the OctreeSegments
        std::vector<std::shared_ptr<Primitive>>* realPrimitives;

        bool IsInsideBounds(Bounds3f bounds);
        bool IntersectVector(const Ray& ray, SurfaceInteraction* isect, const std::vector<int>* primList) const;

    };

	public:

          OwOAccel(std::vector<std::shared_ptr<Primitive>> p);
          Bounds3f WorldBound() const;
          ~OwOAccel();
          bool Intersect(const Ray &ray, SurfaceInteraction *isect) const;
          bool IntersectP(const Ray &ray) const;



    private:        
        float CalculateIntersectDepth(Point3f isectPoint, Point3f rayOrigin) const;
        std::vector<std::shared_ptr<Primitive>> primitives;
        Bounds3f bounds;

        float gridSize = 1;
        OctreeSegment root;

};

std::shared_ptr<OwOAccel> CreateOwOAccelerator(std::vector<std::shared_ptr<Primitive>> prims, const ParamSet &ps);
}
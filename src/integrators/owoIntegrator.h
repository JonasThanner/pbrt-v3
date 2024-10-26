
// integrators/whitted.h*
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"

namespace pbrt {

// WhittedIntegrator Declarations
class OwOIntegrator : public SamplerIntegrator {
  public:
    // WhittedIntegrator Public Methods
    OwOIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
                      std::shared_ptr<Sampler> sampler,
                      const Bounds2i &pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}
    Spectrum Li(const RayDifferential &ray, const Scene &scene,
                Sampler &sampler, MemoryArena &arena, int depth) const;

  private:
    // WhittedIntegrator Private Data
    const int maxDepth;
};

OwOIntegrator *CreateOwOIntegrator(
    const ParamSet &params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);

}  // namespace pbrt
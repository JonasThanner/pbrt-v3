

#include "integrators/owoIntegrator.h"
#include "interaction.h"
#include "camera.h"
#include "film.h"
#include "paramset.h"

namespace pbrt
{



    Spectrum OwOIntegrator::Li(const RayDifferential& ray, const Scene& scene,
                           Sampler& sampler, MemoryArena& arena,
                           int depth) const 
    {
        //Set Profiler Phase
        ProfilePhase p(Prof::SamplerIntegratorLi);

        //Create return specturm = dark
        Spectrum returnSpectrum = Spectrum(0.0f);

        //Create Ray Hit and check for lights
        SurfaceInteraction interaction;
        scene.Intersect(ray, &interaction);

        // Go trough all lights and check if they're visible => If yes, add to returnSpectrum
        for each (const auto& light in scene.lights) 
        {
            //Wouldve liked to do this all by myself and do all the multiplications with cross product etc
            //but theres no uniform lightPoint/light sample for all lights, only the complete  Sample_Li()
            VisibilityTester visTester;
            Vector3f lightSourceDirection;
            Float pdf;
            VisibilityTester visibility;
            Spectrum output = light->Sample_Li(interaction, sampler.Get2D(), &lightSourceDirection, &pdf,
                                           &visibility);

            if (output.IsBlack() || output.HasNaNs())
            {
                continue;
            }
            if (!output.IsBlack() && visibility.Unoccluded(scene))
            {
                //Do cross product calculation for cosine
                returnSpectrum += output * AbsDot(lightSourceDirection, interaction.n) / pdf;
            }
        }



        return returnSpectrum;
    }

    OwOIntegrator* CreateOwOIntegrator(const ParamSet& params,
                                       std::shared_ptr<Sampler> sampler,
                                       std::shared_ptr<const Camera> camera) 
    {
        return new OwOIntegrator(params.FindOneInt("maxdepth", 5), camera,
                                 sampler, camera->film->GetSampleBounds());
    }

    }
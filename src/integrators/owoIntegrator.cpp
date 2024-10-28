

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

        //Create return specturm = dark
        Spectrum returnSpectrum = Spectrum(0.0f);

        //Create Ray Hit and check for lights
        SurfaceInteraction interaction;
        if (!scene.Intersect(ray, &interaction))
        {
            return returnSpectrum;
        }

        //Grab a bsdf and if one doesnt exist => return empty/black area
        interaction.ComputeScatteringFunctions(ray, arena);
        if (!interaction.bsdf)
        {
            return returnSpectrum;
        }

        // Go trough all lights and check if they're visible => If yes, add to returnSpectrum
        for each (const auto& light in scene.lights) 
        {
            //Wouldve liked to do this all by myself and do all the multiplications with cosine etc
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
            
            Spectrum matColor = interaction.bsdf->f(interaction.wo, lightSourceDirection);

            if (!matColor.IsBlack() && !output.IsBlack() && visibility.Unoccluded(scene) && pdf != 0)
            {
                //Do cosine calculations
                returnSpectrum += matColor * output * AbsDot(lightSourceDirection, interaction.shading.n) / pdf;
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
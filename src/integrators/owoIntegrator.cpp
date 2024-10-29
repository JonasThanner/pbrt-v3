

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

        if (depth < 10)
        {
            depth++;

            //Calculate the reflectance ray
            RayDifferential reflectRay = CalculateReflectanceRay(ray, interaction, scene);

            //Get the bsdf reflectance brightness with the reflectance ray
            BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
            Spectrum reflectanceBrightness = interaction.bsdf->f(interaction.wo, reflectRay.d, type);

            if (!reflectanceBrightness.IsBlack())
            {
                returnSpectrum += Li(reflectRay, scene, sampler, arena, depth);
            }
            

        }





        return returnSpectrum;
    }

    RayDifferential OwOIntegrator::CalculateReflectanceRay(const RayDifferential &ray, const SurfaceInteraction &isect, const Scene &scene) const
    {
        //For now lets do perfect reflectance based on bsdf refelectance
        
        //First lets get the bsdf
        BSDF* bsdf = isect.bsdf;

        //Calculate the reflected ray
        Vector3f normalizedRay = ray.d / ray.d.Length();
        Vector3f reflectedRay = ray.d - 2 * Dot(normalizedRay, isect.n) * Vector3f(isect.n.x, isect.n.y, isect.n.z);
        
        return RayDifferential(Point3f(isect.wo.x, isect.wo.y, isect.wo.z), reflectedRay / reflectedRay.Length());
    }

    OwOIntegrator* CreateOwOIntegrator(const ParamSet& params,
                                       std::shared_ptr<Sampler> sampler,
                                       std::shared_ptr<const Camera> camera) 
    {
        return new OwOIntegrator(params.FindOneInt("maxdepth", 5), camera,
                                 sampler, camera->film->GetSampleBounds());
    }

}
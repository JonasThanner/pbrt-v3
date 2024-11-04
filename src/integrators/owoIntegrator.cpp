

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
        SurfaceInteraction intersection;
        if (!scene.Intersect(ray, &intersection))
        {
            return returnSpectrum;
        }

        //Grab a bsdf and if one doesnt exist => return empty/black area
        intersection.ComputeScatteringFunctions(ray, arena);
        if (!intersection.bsdf)
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
            Spectrum output = light->Sample_Li(intersection, sampler.Get2D(), &lightSourceDirection, &pdf,
                                           &visibility);

            if (output.IsBlack() || output.HasNaNs())
            {
                continue;
            }
            
            Spectrum matColor = intersection.bsdf->f(intersection.wo, lightSourceDirection);

            if (!matColor.IsBlack() && !output.IsBlack() && visibility.Unoccluded(scene) && pdf != 0)
            {
                //Do cosine calculations
                returnSpectrum += matColor * output * AbsDot(lightSourceDirection, intersection.shading.n) / pdf;
            }
        }

        if (depth < 10)
        {
            depth++;

            //Calculate the reflectance ray
            RayDifferential reflectRay = CalculateReflectanceRay(ray, intersection, scene);

            //Get the bsdf reflectance brightness with the reflectance ray
            BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
            Spectrum reflectanceBrightness = intersection.bsdf->f(intersection.wo, reflectRay.d, type);

            Vector3f wo = intersection.wo, wi;
            Float pdf;



            Spectrum f = intersection.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, type);

            //Vector3f wi_local = intersection.bsdf->WorldToLocal(reflectRay.d);
            //Vector3f wo_local = intersection.bsdf->WorldToLocal(wo);




            /*reflectanceBrightness = intersection.bsdf->f(wo_local, wi_local);*/
            if (!f.IsBlack())
            {
                returnSpectrum += Li(intersection.SpawnRay(wi), scene, sampler, arena, depth);
            }

            //returnSpectrum += SpecularReflect(ray, interaction, scene, sampler, arena, depth);
            

        }





        return returnSpectrum;
    }

    RayDifferential OwOIntegrator::CalculateReflectanceRay(const RayDifferential &ray, const SurfaceInteraction &isect, const Scene &scene) const
    {
        //For now lets do perfect reflectance based on bsdf refelectance

        //Calculate the reflected ray
        Vector3f normalizedRay = ray.d / ray.d.Length();
        Vector3f reflectedRay = - ray.d - Vector3f( 2 * Dot(isect.n, -normalizedRay) * isect.n);
        
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
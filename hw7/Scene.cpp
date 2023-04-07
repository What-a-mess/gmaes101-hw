//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <iostream>


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                pos.obj = objects[k];
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Vector3f dir_light = Vector3f(0, 0, 0), indir_light = Vector3f(0, 0, 0);
    Vector3f wo = (-ray.direction).normalized();

    auto format = [](Vector3f &a) {
        if(a.x < 0) a.x = 0;
        if(a.y < 0) a.y = 0;
        if(a.z < 0) a.z = 0;
    };

    Intersection intersection = Scene::intersect(ray);
    if (intersection.happened) {
        if (!(intersection.emit.norm() > 1e-2)) {

            Intersection light_pos;
            float light_pdf;
            sampleLight(light_pos, light_pdf);
            
            Intersection test_intersection;
            Ray test_ray = Ray(intersection.coords, (light_pos.coords-intersection.coords).normalized());
            test_intersection = Scene::intersect(test_ray);

            // printf("%f\n", (test_intersection.coords-light_pos.coords).norm());
            if (test_intersection.happened && (test_intersection.coords-light_pos.coords).norm() < 1e-2) {
                // printf("light!\n");
                dir_light = light_pos.emit 
                                * intersection.m->eval(test_ray.direction, wo, intersection.normal)
                                * dotProduct(intersection.normal, test_ray.direction)
                                * dotProduct(-test_ray.direction, light_pos.normal)
                                / dotProduct(intersection.coords-light_pos.coords, intersection.coords-light_pos.coords)
                                / light_pdf;

                // std::cout << intersection.m->eval(test_ray.direction, ray.direction, intersection.normal) << std::endl;

                // if (dir_light.norm() < 1e-2) {
                //     std::cout << light_pos.emit << "\n* " << intersection.m->eval(test_ray.direction, ray.direction, intersection.normal) << "\n* " << dotProduct(test_ray.direction, light_pos.normal) << "\n* ";
                // }
            }
        } else if (depth == 0) {
            dir_light = intersection.emit;
        }
        if (get_random_float() < RussianRoulette) {
            Vector3f wi = intersection.m->sample(wo, intersection.normal);
            indir_light = castRay(Ray(intersection.coords, wi), depth+1)
                            * intersection.m->eval(wi, wo, intersection.normal)
                            * dotProduct(intersection.normal, wi)
                            / intersection.m->pdf(wi, wo, intersection.normal)
                            / RussianRoulette;

        }

    }
    
    return dir_light + indir_light;
}
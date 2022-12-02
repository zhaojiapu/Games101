//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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
// 伪代码：
//  shade(p, wo)
//      sampleLight(inter, pdf_light)
//      Get x, ws, NN, emit from inter
//      Shoot a ray from p to x
//      If the ray is not blocked in the middle
//          L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
//
//      L_indir = 0.0
//      Test Russian Roulette with probability RussianRoulette wi = sample(wo, N)
//      Trace a ray r(p, wi)
//      If ray r hit a non-emitting object at q
//          L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
//
//      Return L_dir + L_indir
//
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // 在其中实现 Path Tracing 算法 可能用到的函数有:
    //
    // intersect(const Ray ray)in Scene.cpp: 求一条光线与场景的交点
    //
    // sampleLight(Intersection pos, float pdf) in Scene.cpp:
    //  在场景的所有光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度 3
    //
    // sample(const Vector3f wi, const Vector3f N) in Material.cpp:
    //  按照该材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
    //
    // pdf(const Vector3f wi, const Vector3f wo, const Vector3f N) in Material.cpp:
    //  给定一对入射、出射方向与法向量，计算 sample 方法得到该出射方向的概率密度
    //
    // eval(const Vector3f wi, const Vector3f wo, const Vector3f N) in Material.cpp:
    //  给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值
    //
    // 可能用到的变量有:
    //
    // RussianRoulette in Scene.cpp:
    //  P_RR, Russian Roulette 的概率
    //
    // TO DO Implement Path Tracing Algorithm here

    // 光线与光源直接相交
    Intersection Pinter = intersect(ray);
    if (!Pinter.happened) {
        return Vector3f();
    }

    if (Pinter.m->hasEmission()) {
        return Pinter.m->getEmission();
    }

    // 光线与物体相交
    const float EPSILON = 0.0001;
    Vector3f l_dir;
    Vector3f l_indir;

    // 光源发出的光线
    Intersection Xinter;
    float pdf_light = 0.0f;
    sampleLight(Xinter, pdf_light);

    Vector3f p = Pinter.coords;
    Vector3f x = Xinter.coords;
    Vector3f ws_dir = (x - p).normalized();
    float ws_dis = (x - p).norm();
    Vector3f N = Pinter.normal.normalized();
    Vector3f NN = Xinter.normal.normalized();
    Vector3f emit = Xinter.emit;

    Ray ws_ray(p, ws_dir);
    Intersection ws_ray_inter = intersect(ws_ray);

    if (ws_ray_inter.distance - ws_dis > -EPSILON) {
        l_dir = emit * Pinter.m->eval(ray.direction, ws_ray.direction, N)
                * dotProduct(ws_ray.direction, N)
                * dotProduct(-ws_ray.direction, NN)
                / std::pow(ws_dis, 2)
                / pdf_light;
    }

    // 俄罗斯轮盘赌，决定是否反射光线
    if (get_random_float() > RussianRoulette) {
        return l_dir;
    }

    // 反射光，需要递归计算
    Vector3f wi_dir = Pinter.m->sample(ray.direction, N).normalized();
    Ray wi_ray(Pinter.coords, wi_dir);

    Intersection Winter = intersect(wi_ray);

    if (Winter.happened && (!Winter.m->hasEmission())) {
        l_indir = castRay(wi_ray, depth + 1)
                  * Pinter.m->eval(ray.direction, wi_ray.direction, N)
                  * dotProduct(wi_ray.direction, N)
                  / Pinter.m->pdf(ray.direction, wi_ray.direction, N)
                  / RussianRoulette;
    }

    return l_dir + l_indir;
}
#pragma once

#include "main/scene_base/base.hpp"

#ifdef POOL_GAME

struct aabb
{
    aabb(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, vcl::vec3 normal);
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
    vcl::vec3 normal;
};

// Structure of a particle
struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces

    vcl::vec3 c; // Color
    float r;     // Radius
};

struct gui_scene_structure
{
    bool add_sphere = true;
    float time_interval_new_sphere = 0.5f;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void setup_aabb(std::map<std::string, GLuint>& shaders);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt);
    void create_new_particle();
    void triangle_base_configuration();
    void display_particles(scene_structure& scene);

    // Textures
    GLuint texture_green;
    GLuint texture_wood;
    std::vector<particle_structure> particles;

    std::vector<aabb> boundaries;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::mesh_drawable ground;
    vcl::mesh pool;
    vcl::mesh_drawable pool_visual;
    vcl::segments_drawable borders; // Visual display of borders

    vcl::timer_event timer;
    gui_scene_structure gui_scene;

    float radius_ball = 0.03f;

};






#endif
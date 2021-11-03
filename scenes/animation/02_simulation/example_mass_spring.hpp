#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_MASS_SPRING_1D

struct particle_element
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
};

struct user_parameters_structure
{
    float m;    // Global mass (to be divided by the number of particles)
    float K;    // Global stiffness (to be divided by the number of particles)
    float mu;   // Damping
    float wind; // Wind magnitude;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);


    std::vector<particle_element> points;
    size_t nb_points = 10;
    particle_element pA;
    particle_element pB;
    particle_element pC;
    float L0;

    user_parameters_structure user_parameters;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders
    vcl::segment_drawable_immediate_mode segment_drawer;

    vcl::timer_event timer;

    void set_gui(vcl::timer_basic &timer);
};






#endif

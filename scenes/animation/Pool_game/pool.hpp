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

    void compute_time_step(float dt, float partial_alpha, float partial_beta);
    void check_collisions(float partial_alpha, float partial_beta);

    void white_ball_setup();
    void triangle_base_configuration();
    void display_particles(scene_structure& scene);

    void mouse_click(scene_structure& scene, GLFWwindow* window, int button, int action, int mods);

    bool check_state();
    void check_score();
    void check_white_ball();

    void update_camera(scene_structure& scene);

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

    int steps_in_frame = 10;
    float max_speed = 10.f;
    float alpha = .985f; // Restitution coefficient in parallel direction (friction)
    float beta = .985f; // Restitution coefficient in orthogonal direction (impact)

    int score = 0;
    bool play_allowed = true;

    bool is_throwing = false;
    vcl::vec3 throw_pos = vcl::vec3(0, 0, 0);

};

#endif
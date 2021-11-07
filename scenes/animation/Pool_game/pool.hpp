#pragma once

#include "main/scene_base/base.hpp"
#include "textRender.hpp"

#ifdef POOL_GAME

struct aabb
{
    aabb(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, bool bouncing);
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
    bool bouncing;
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

struct animation_camera
{
    float time_animation_begin = 0.0f;
    float animation_duration = 1.0f;

    float theta_begin;
    float phi_begin;
    float theta_end;
    float phi_end = -0.45;

    vcl::vec3 p_save;
    vcl::vec3 p_begin;
    vcl::vec3 p_end = vcl::vec3{0, 0, -0.8};

    vcl::vec2 interpolate_camera_position(float t) const;

    vcl::vec3 interpolate_reference_position(float t) const;
    static float angle_between_center_and_point(const vcl::vec3 &point);
    static void update_camera(scene_structure& scene, vcl::vec3 position, vcl::vec2 spherical_coordinates);
};

struct scene_model : scene_base
{
    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void setup_aabb(std::map<std::string, GLuint>& shaders);
    void draw_objects(std::map<std::string,GLuint>& shaders, scene_structure& scene);
    void draw_cue(std::map<std::string,GLuint>& shaders, scene_structure& scene);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt, float partial_alpha, float partial_beta);
    void check_collisions(float partial_alpha, float partial_beta);

    void white_ball_setup();
    void triangle_base_configuration();
    void display_particles(scene_structure& scene);

    void mouse_click(scene_structure& scene, GLFWwindow* window, int button, int action, int mods);
    void mouse_move(scene_structure& scene, GLFWwindow* window);

    bool check_state();
    void check_score();
    void check_white_ball();
    void check_end_game();


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

    bool reset = false;
    float radius_ball = 0.03f;
    vcl::vec3 white_ball_position = vcl::vec3{0,0,0.8};

    int steps_in_frame = 10;
    float max_speed = 7.5f;
    float alpha = .985f; // Restitution coefficient in parallel direction (friction)
    float beta = .985f; // Restitution coefficient in orthogonal direction (impact)

    unsigned int score = 0;
    unsigned int nb_shots = 0;
    bool play_allowed = true;
    bool in_animation = true;
    bool gameFinished = false;
    bool animateCamera = true;
    animation_camera animationCamera;

    bool is_throwing = false;
    vcl::vec3 throw_pos = vcl::vec3(0, 0, 0);

    float distance = 0;
    vcl::vec3 throw_dir = vcl::vec3(0, 0, 0);

    textRender textRenderer;

};

#endif

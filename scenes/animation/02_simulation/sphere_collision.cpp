
#include "sphere_collision.hpp"

#include <random>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;







void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    create_new_particle();
    compute_time_step(dt);

    display_particles(scene);
    draw(borders, scene.camera);

}

void scene_model::compute_time_step(float dt)
{
    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,-9.81f,0);


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v = (1-0.9f*dt) * v + dt * f; // gravity + friction force
        p = p + dt * v;
    }

    // Collisions with cube
    const float alpha = 0.8f; // Restitution coefficient in parallel direction (friction)
    const float beta = 0.8f; // Restitution coefficient in orthogonal direction (impact)
    const float epsilon = 0.2f;
    const float mu = 0.5f;
    // Collisions between spheres
    for (size_t k = 0; k < N; ++k) {
      particle_structure& particle_1 = particles[k];
      vec3& v1 = particle_1.v;
      vec3& p1 = particle_1.p;
      float r1 = particle_1.r;
      for (size_t j = 0; j < N; ++j) {
        particle_structure& particle_2 = particles[j];
        vec3& p2 = particle_2.p;
        float r2 = particle_2.r;
        if (j == k || norm(p1 - p2) > r1 + r2)
          continue;
        vec3& v2 = particle_2.v;
        vec3 u = (p1 - p2) / norm(p1 - p2);
        // Static contact
        if (norm(v1 - v2) < epsilon) {
          v1 = mu * v1;
          v2 = mu * v2;
        } else {
          vec3 v1_orthogonal = dot(v1, u) * u;
          vec3 v1_parallel = v1 - v1_orthogonal;
          vec3 v2_orthogonal = dot(v2, u) * u;
          vec3 v2_parallel = v2 - v2_orthogonal;

          v1 = alpha * v1_parallel + beta * v2_orthogonal;
          v2 = alpha * v2_parallel + beta * v1_orthogonal;
        }

        float distance_penetration = r1 + r2 - norm(p1 - p2);
        p1 += (distance_penetration / 2) * u;
        p2 += (distance_penetration / 2) * -u;

      }
    }

    std::vector<vec3> normals_cube = {{0,1,0},{0,-1,0}, {1,0,0},{-1,0,0},
                                      {0,0,1},{0,0,-1}};
    std::vector<vec3> points_cube = {{-1,-1,-1}, {1,1,1}, {-1,-1,-1}, {1,-1,-1},
                                     {-1,-1,-1}, {-1,-1,1}};
    for (size_t k = 0; k < N; ++k) {
      particle_structure& particle = particles[k];
      vec3& v = particle.v;
      vec3& p = particle.p;
      float r = particle.r;
      for (size_t j = 0; j < 6; ++j) {
        vec3 a = points_cube[j];
        vec3 n = normals_cube[j];
        float detection = dot(p - a, n);

        if (detection <= r) {
          vec3 v_orthogonal = dot(v, n) * n;
          vec3 v_parallel = v - v_orthogonal;
          v = alpha * v_parallel - beta * v_orthogonal;

          p += n * (r - dot(p - a, n));
        }
      }

    }



}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);

    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}





#endif


#include "example_mass_spring.hpp"


#ifdef SCENE_MASS_SPRING_1D

using namespace vcl;


/** Compute spring force applied on particle pi from particle pj */
vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
    vec3 const pji = pj - pi;
    float const L = norm(pji);
    return K * (L - L0) * pji / L;
}


void scene_model::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& )
{
    // Initial position and speed of particles
    // ******************************************* //
    for (size_t i = 0; i < nb_points; ++i) {
        particle_element point = particle_element();
        point.p = {i * 0.5f, 0, 0};
        point.v = {0, 0, 0};
        points.emplace_back(point);
    }
    pA.p = {0,0,0};     // Initial position of particle A
    pB.v = {0,0,0};     // Initial speed of particle A

    pB.p = {0.5f,0,0};  // Initial position of particle B
    pB.v = {0,0,0};     // Initial speed of particle B

    pC.p = {1.0f,0,0};  // Initial position of particle B
    pC.v = {0,0,0};     // Initial speed of particle B

    L0 = 0.4f; // Rest length between A and B


    // Display elements
    // ******************************************* //
    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0,0,1};

    sphere = mesh_primitive_sphere();
    sphere.uniform.transform.scaling = 0.05f;


    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = borders_segments;
    borders.uniform.color = {0,0,0};

// Default value for simulation parameters
    user_parameters.K    = 7.0f;
    user_parameters.m    = 0.06f;
    user_parameters.mu   = 0.01f;

}





void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui(timer);


  // Simulation parameters
    const float m  = user_parameters.m;        // particle mass
    const float K  = user_parameters.K;         // spring stiffness
    const float mu = user_parameters.mu;       // damping coefficient

    const vec3 g   = {0,-9.81f,0}; // gravity

    // Simulation time step (dt)
    float dt = timer.scale*0.01f;

    bool generalized = true;
    if (generalized) {
        for (size_t i = 1; i < nb_points; ++i) {
            vec3 f_spring_up = spring_force(points[i].p, points[i - 1].p, L0, K);
            vec3 f_spring_down = {0, 0, 0};
            if (i != nb_points - 1)
                f_spring_down = spring_force(points[i].p, points[i + 1].p, L0, K);

            vec3 f_weight = m * g;
            vec3 F = f_spring_up + f_spring_down + f_weight;

            vec3 &p = points[i].p; // position of particle
            vec3 &v = points[i].v; // speed of particle

            v = v + dt * F / m;
            v *= (1 - mu);
            p = p + dt * v;

        }
        for (size_t i = 0; i < nb_points; ++i) {
            sphere.uniform.transform.translation = points[i].p;
            sphere.uniform.color = ((float)i / (float)nb_points) * vec3{1, 1, 1};
            draw(sphere, scene.camera, shaders["mesh"]);

            if (i != nb_points - 1) {
                segment_drawer.uniform_parameter.p1 = points[i].p;
                segment_drawer.uniform_parameter.p2 = points[i + 1].p;
                segment_drawer.draw(shaders["segment_im"], scene.camera);
            }
        }
    } else {

        // Forces
        vec3 f_spring = spring_force(pC.p, pB.p, L0, K);
        vec3 f_weight = m * g;
        vec3 F_1 = f_spring + f_weight;

        {
            vec3 &p = pC.p; // position of particle
            vec3 &v = pC.v; // speed of particle

            v = v + dt * F_1 / m;
            v *= (1 - mu);
            p = p + dt * v;
        }
        f_spring = spring_force(pB.p, pA.p, L0, K);
        F_1 = f_spring + f_weight;

        f_spring = spring_force(pB.p, pC.p, L0, K);
        vec3 F_2 = f_spring;
        // Numerical Integration (Verlet)
        {
            // Only particle B should be updated
            vec3 &p = pB.p; // position of particle
            vec3 &v = pB.v; // speed of particle

            v = v + dt * (F_1 + F_2) / m;
            v *= (1 - mu);
            p = p + dt * v;
        }


        // Display of the result


        // particle pa
        sphere.uniform.transform.translation = pA.p;
        sphere.uniform.color = {0, 0, 0};
        draw(sphere, scene.camera, shaders["mesh"]);

        // particle pb
        sphere.uniform.transform.translation = pB.p;
        sphere.uniform.color = {1, 0, 0};
        draw(sphere, scene.camera, shaders["mesh"]);

        // particle pc
        sphere.uniform.transform.translation = pC.p;
        sphere.uniform.color = {0, 1, 0};
        draw(sphere, scene.camera, shaders["mesh"]);


        // Spring pa-pb
        segment_drawer.uniform_parameter.p1 = pA.p;
        segment_drawer.uniform_parameter.p2 = pB.p;
        segment_drawer.draw(shaders["segment_im"], scene.camera);

        // Spring pb-pc
        segment_drawer.uniform_parameter.p1 = pB.p;
        segment_drawer.uniform_parameter.p2 = pC.p;
        segment_drawer.draw(shaders["segment_im"], scene.camera);
    }


  draw(borders, scene.camera, shaders["curve"]);
}


/** Part specific GUI drawing */
void scene_model::set_gui(timer_basic& timer)
{
    ImGui::SliderFloat("Damping", &user_parameters.mu, 0.0f, 0.1f, "%.3f s");
    ImGui::SliderFloat("K", &user_parameters.K, 1.0f, 10.0f, "%.3f s");
    ImGui::SliderFloat("Mass", &user_parameters.m, 0.001f, 0.1f, "%.3f s");
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();

}



#endif

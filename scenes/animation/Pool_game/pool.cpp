
#include "pool.hpp"

#include <random>

#ifdef POOL_GAME

using namespace vcl;

void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    create_new_particle();
    compute_time_step(dt);

    display_particles(scene);
    //draw(ground, scene.camera);
    draw(borders, scene.camera);

    //draw(pool_visual, scene.camera);
    //glBindTexture(GL_TEXTURE_2D, scene.texture_white);

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
    const float alpha = 0.95; // Restitution coefficient in parallel direction (friction)
    const float beta = 0.95; // Restitution coefficient in orthogonal direction (impact)
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


    std::vector<vec3> normals_cube = {{0,1,0}};
    std::vector<vec3> points_cube = {{0,0,0}};
    //buffer<vec3> points_cube = pool.position;
    //buffer<vec3> normals_cube = pool.normal;

    // Code to check intersection with finite object using aabb
    for (size_t k = 0; k < N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        float r = particle.r;
        for (size_t j = 0; j < boundaries.size(); ++j) {
            aabb box = boundaries[j];
            float x = std::max(box.minX, std::min(p.x, box.maxX));
            float y = std::max(box.minY, std::min(p.y, box.maxY));
            float z = std::max(box.minZ, std::min(p.z, box.maxZ));
            vec3 closest_point_aabb = vec3{x, y, z};

            float distance = vcl::norm(p - closest_point_aabb);
            if (distance <= r) {
                vec3 v_orthogonal = dot(v, box.normal) * box.normal;
                vec3 v_parallel = v - v_orthogonal;
                v = alpha * v_parallel - beta * v_orthogonal;

                p += box.normal * (r - dot(p - closest_point_aabb, box.normal));
            }
        }


    }

    // Code to check intersection with infinite plane
    /*
    size_t nb_faces_to_check = points_cube.size();
    for (size_t k = 0; k < N; ++k) {
      particle_structure& particle = particles[k];
      vec3& v = particle.v;
      vec3& p = particle.p;
      float r = particle.r;
      for (size_t j = 0; j < nb_faces_to_check; ++j) {
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
     */



}

// Use this function to create the triangle and erase the balls still present in the scene
void scene_model::triangle_base_configuration() {
    particles.clear();
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};
    float diameter_ball = 2.0f * radius_ball;
    float offset_z = std::sqrt(3.0f) * radius_ball; // Found by looking for the z that minimizes the norm between 2 balls
    vec3 offset_triangle = vec3{-2.0f * diameter_ball, 0.0f, -1.0f};
    for (size_t i = 0; i < 5; ++i) {
        for (size_t j = 0; j < 5 - i; ++j) {
            particle_structure new_ball;
            new_ball.r = radius_ball;
            new_ball.c = color_lut[int(rand_interval()*color_lut.size())];
            new_ball.p = vec3(diameter_ball * (j + i / 2.0f), 0.1f,  i * offset_z) + offset_triangle;
            particles.push_back(new_ball);
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

        new_particle.r = radius_ball;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0.1f,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = 5.0f * normalize(vec3( 3.0f * std::cos(timer.t), 0.0f, 5.0f * std::sin(timer.t)));
        //new_particle.v = 5.0f * normalize(vec3(0.3f, 0.0f, 0.68f));

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


// From a struct aabb returns the list of corners, to use for visualization
std::vector<vec3> segments_from_aabb(const aabb& box) {
    std::vector<vec3> corners{{box.minX, box.minY, box.minZ},
                              {box.maxX, box.minY, box.minZ},
                              {box.maxX, box.minY, box.maxZ},
                              {box.minX, box.minY, box.maxZ},
                              {box.minX, box.maxY, box.minZ},
                              {box.maxX, box.maxY, box.minZ},
                              {box.maxX, box.maxY, box.maxZ},
                              {box.minX, box.maxY, box.maxZ}};
    return std::vector<vec3>{corners[0], corners[1], corners[1], corners[2], corners[2], corners[3], corners[3], corners[0],
                             corners[0], corners[4], corners[1], corners[5], corners[2], corners[6], corners[3], corners[7],
                             corners[4], corners[5], corners[5], corners[6], corners[6], corners[7], corners[7], corners[4]};

}

std::vector<vec3> segments_from_aabbs(const std::vector<aabb>& boxes) {
    std::vector<vec3> segments_boxes;
    for (const auto& box : boxes) {
        std::vector<vec3> segments_box = segments_from_aabb(box);
        segments_boxes.insert(segments_boxes.end(), segments_box.begin(), segments_box.end());
    }
    return segments_boxes;
}

void scene_model::setup_aabb(std::map<std::string, GLuint>& shaders) {
    float x_ground = 0.635;
    float z_ground = 1.27;
    float width_border = 0.1f;
    float height_border = 0.2f;
    float size_hole = 0.1f;
    float corner_hole_bias = 0.1f;
    float width_hole_side = 0.01f;
    // A pool table is symmetric along its z and x axis so we don't need to specify all corners
    std::vector<aabb> corners = {{x_ground - width_hole_side, x_ground, 0, height_border, 0, size_hole / 2.0f, {-1, 0, 0}},
                                 {x_ground - width_border, x_ground - width_hole_side, 0, height_border, size_hole / 2.0f, size_hole / 2.0f, {0, 0, 1}},
                                 {x_ground - width_border, x_ground, 0, height_border, size_hole / 2.0f, z_ground - width_border - corner_hole_bias, {-1, 0, 0}},
                                 {x_ground - width_hole_side, x_ground, 0, height_border, z_ground - width_border - corner_hole_bias, z_ground - width_hole_side, {-1, 0, 0}},
                                 {x_ground - width_border - corner_hole_bias, x_ground - width_hole_side, 0, height_border, z_ground - width_hole_side, z_ground, {0, 0, -1}},
                                 {0, x_ground - width_border - corner_hole_bias, 0, height_border, z_ground - width_border, z_ground, {0, 0, -1}}};
    for (int symmetric_x = -1; symmetric_x <= 1; symmetric_x += 2) {
        for (int symmetric_z = -1; symmetric_z <= 1; symmetric_z += 2) {
            for (auto corner : corners) {
                vec3 normal = vec3(corner.normal.x * symmetric_x, corner.normal.y, corner.normal.z * symmetric_z);
                boundaries.emplace_back(corner.minX * symmetric_x, corner.maxX * symmetric_x, corner.minY, corner.maxY, corner.minZ * symmetric_z, corner.maxZ * symmetric_z, normal);
            }
        }
    }
    boundaries.push_back({-x_ground + width_border, x_ground - width_border, 0, 0, -z_ground + width_border, z_ground - width_border, {0,1,0}});
    float height_under_ground = -1.0f;
    boundaries.push_back({-10.0f, 10.0f, height_under_ground, height_under_ground - 1.0f, -10.0f, 10.0f, {0,1,0}});

    ground = mesh_drawable(mesh_primitive_quad({-x_ground + width_border,0,-z_ground + width_border}, {x_ground - width_border,0,-z_ground + width_border}, {x_ground - width_border,0,z_ground - width_border}, {-x_ground + width_border,0,z_ground - width_border}));

    // This line must be called after boundaries is full
    borders = segments_gpu(segments_from_aabbs((boundaries)));
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

    ground.shader = shaders["mesh_bf"];
    ground.texture_id = texture_green;
}

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    setup_aabb(shaders);
    triangle_base_configuration();

    texture_wood  = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/wood.png"));
    texture_green = create_texture_gpu(image_load_png("scenes/animation/Pool_game/assets/green.png"));
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    this->pool = mesh_load_file_obj("scenes/animation/Pool_game/assets/pool.obj");
    pool_visual = mesh_drawable(pool);
    pool_visual.shader = shaders["mesh_bf"];
    pool_visual.texture_id = texture_wood;
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


aabb::aabb(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, vcl::vec3 normal) {
    if (minX <= maxX) {
        this->minX = minX;
        this->maxX = maxX;
    } else {
        this->minX = maxX;
        this->maxX = minX;
    }
    if (minY <= maxY) {
        this->minY = minY;
        this->maxY = maxY;
    } else {
        this->minY = maxY;
        this->maxY = minY;
    }
    if (minZ <= maxZ) {
        this->minZ = minZ;
        this->maxZ = maxZ;
    } else {
        this->minZ = maxZ;
        this->maxZ = minZ;
    }
    this->normal = normalize(normal);
}

#endif
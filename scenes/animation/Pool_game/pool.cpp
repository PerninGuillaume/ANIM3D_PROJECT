#include <random>

#include "pool.hpp"

#ifdef POOL_GAME

using namespace vcl;

static float linear_interpolation(float t, float t_final, float angle1, float angle2);
static vec3 linear_interpolation(float t, float t_final, vec3 p1, vec3 p2);

// This functions reduces the given radian to one that is in the range [0,2*PI]
static float get_positive_radian(float radian) {
    return fmod(fmod(radian, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
}

// We want to avoid multiples useless rotations when the animation of the camera plays
// This methods ensures the rotations don't go over 180 degrees
static void smallest_rotations(float& radian_begin, float& radian_end) {
    if (std::fabs(radian_end - radian_begin) < M_PI) // if angle is already the smallest possible
        return;
    if (radian_end > radian_begin)
        radian_end -= 2 * M_PI;
    else
        radian_begin -= 2 * M_PI;
}

void print_score(textRender& textRenderer, unsigned int score, unsigned int nb_shots) {
    textRenderer.renderText("Score : " + std::to_string(score), 1750.0f, 1050.0f, 1.0f, vec3(0.0, 0.0, 0.0));
    textRenderer.renderText("Number of shots : " + std::to_string(nb_shots), 1650.0f, 1000.0f, 1.0f, vec3(0.0, 0.0, 0.0));
}

void print_end_game(textRender& textRenderer, unsigned int nb_shots) {
    std::map<unsigned int, std::string> win_messages = {{16, "Congratulation ! You must be either a god or a cheater"},
                                               {24, "Congratulation ! You mastered the game !"},
                                               {36, "Congratulation ! Your score is great !"},
                                               {45, "Congratulation ! You can do better !"},
                                               {60, "Come on, try to aim next time"},
                                               {70, "Are you even trying ?"}};
    auto it = win_messages.upper_bound(nb_shots);
    std::string message;
    if (it == win_messages.end())
        message = "What happened ? A special case in the code had to be written just because of people like you";
    else
        message = it->second;

    textRenderer.renderText("Number of shots : " + std::to_string(nb_shots), 1650.0f, 1000.0f, 1.0f, vec3(0.0, 0.0, 0.0));
    textRenderer.renderText(message, 700.0f, 500.0f, 1.0f, vec3(0.0, 0.0, 0.0));
}

void scene_model::draw_objects(std::map<std::string,GLuint>& shaders, scene_structure& scene) {
    draw(pool_visual, scene.camera);
    draw(plane_pool_visual, scene.camera);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white); // To reset the texture
    display_particles(scene);

    if (display_aabb)
        draw(borders, scene.camera);

    if (not in_animation)
        draw_cue(shaders, scene);

    if (gameFinished)
        print_end_game(textRenderer, nb_shots);
    else
        print_score(textRenderer, score, nb_shots);

}

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& ) {
    float dt = 0.02f * timer.scale;
    timer.update();
    float t = timer.t;
    in_animation = animateCamera && (t - animationCamera.time_animation_begin < animationCamera.animation_duration);

    set_gui();

    if (in_animation) {
        animation_camera::update_camera(scene, animationCamera.interpolate_reference_position(t),
                                      animationCamera.interpolate_camera_position(t));
        draw_objects(shaders, scene);
    }

    float partial_alpha = std::pow(alpha, 1.0/steps_in_frame);
    float partial_beta = std::pow(beta, 1.0/steps_in_frame);
    for (int i = 0; i < steps_in_frame; ++i) {
        compute_time_step(dt / steps_in_frame, partial_alpha, partial_beta);
    }

    draw_objects(shaders, scene);

    if (check_state() && !play_allowed) // We enter here on the frame the action finishes
    {
        play_allowed = true;
        animationCamera.time_animation_begin = timer.t;

        animationCamera.theta_begin = get_positive_radian(scene.camera.spherical_coordinates.x);
        animationCamera.phi_begin = get_positive_radian(scene.camera.spherical_coordinates.y);
        animationCamera.theta_end = get_positive_radian(animation_camera::angle_between_center_and_point(particles[0].p));
        smallest_rotations(animationCamera.theta_begin, animationCamera.theta_end);
        smallest_rotations(animationCamera.phi_begin, animationCamera.phi_end);

        animationCamera.p_begin = animationCamera.p_save;
        animationCamera.p_end = -particles[0].p;
        animationCamera.p_save = -particles[0].p;
        if (!animateCamera)
            animation_camera::update_camera(scene, animationCamera.p_end, vec2{animationCamera.theta_begin, animationCamera.phi_begin});
    }

    check_score();
    check_white_ball();
    check_end_game();
}

vcl::vec2 animation_camera::interpolate_camera_position(float t) const {
    float theta_intermediate = linear_interpolation(t - time_animation_begin, animation_duration, theta_begin, theta_end);
    float phi_intermediate = linear_interpolation(t - time_animation_begin, animation_duration, phi_begin, phi_end);
    return {theta_intermediate, phi_intermediate};
}

vcl::vec3 animation_camera::interpolate_reference_position(float t) const {
    vec3 p = linear_interpolation(t - time_animation_begin, animation_duration, p_begin, p_end);
    return p;
}

// Ideas from https://learnopengl.com/In-Practice/2D-Game/Collisions/Collision-resolution
// This functions helps to determine the normal along which the collisions happens on an aabb
vcl::vec3 aabb_vector_direction(const vcl::vec3& aabb_vector) {
    const size_t sizeCompass = 6;
    vec3 compass[sizeCompass] = {
            vec3(0.0f, 1.0f, 0.0f),	// y+
            vec3(0.0f, -1.0f, 0.0f),	// y-
            vec3(1.0f, 0.0f, 0.0f),	// x+
            vec3(-1.0f, 0.0f, 0.0f),	// x-
            vec3(0.0f, 0.0f, 1.0f),	// z+
            vec3(0.0f, 0.0f, -1.0f)	// z+
    };
    float dot_max = 0.0f;
    size_t i_max = 0;
    for (size_t i = 0; i < sizeCompass; ++i) {
        float dot_tmp = dot(compass[i], aabb_vector);
        if (dot_tmp > dot_max) {
            dot_max = dot_tmp;
            i_max = i;
        }
    }
    return compass[i_max];
}

void scene_model::check_collisions(float partial_alpha, float partial_beta) {
    const size_t N = particles.size();
    const float epsilon = 0.0f;
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

                v1 = partial_alpha * v1_parallel + partial_beta * v2_orthogonal;
                v2 = partial_alpha * v2_parallel + partial_beta * v1_orthogonal;
            }

            float distance_penetration = r1 + r2 - norm(p1 - p2);
            p1 += (distance_penetration / 2) * u;
            p2 += (distance_penetration / 2) * -u;

        }
    }

    // Inspired from https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection
    // Code to check intersection with finite object using aabb
    for (size_t k = 0; k < N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        float r = particle.r;
        for (auto box : boundaries) {
            float x = std::max(box.minX, std::min(p.x, box.maxX));
            float y = std::max(box.minY, std::min(p.y, box.maxY));
            float z = std::max(box.minZ, std::min(p.z, box.maxZ));
            vec3 closest_point_aabb = vec3{x, y, z};

            vec3 aabb_vector = p - closest_point_aabb;
            float distance_ = vcl::norm(aabb_vector);
            if (distance_ <= r) {
                vec3 normal = aabb_vector_direction(aabb_vector);
                vec3 v_orthogonal = dot(v, normal) * normal;
                vec3 v_parallel = v - v_orthogonal;

                v = partial_alpha * v_parallel - partial_beta * v_orthogonal;
                v.y = std::min(v.y, 0.0f);
                if (!box.bouncing) {
                    v.y = -2.0f; // Prevent balls touching the inside of one hole and bouncing back
                }

                p += normal * (r - dot(aabb_vector, normal));
            }
        }
    }
}

void scene_model::compute_time_step(float dt, float partial_alpha, float partial_beta) {
    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,-9.81f,0);

    // Integrate position and speed of particles through time
    for (size_t k = 0; k < N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v += dt * f; // gravity + friction force
        if (v.y > 0)
            v.y *= 0.98f; // prevent bouncing balls
        p = p + dt * v;
    }

    check_collisions(partial_alpha, partial_beta);
}

void scene_model::white_ball_setup() {
    particle_structure white_ball;

    white_ball.r = radius_ball;
    white_ball.c = vec3(1, 1, 1);
    white_ball.p = white_ball_position;

    particles.insert(particles.begin(), white_ball);
}

// Use this function to create the triangle and erase the balls still present in the scene
void scene_model::triangle_base_configuration() {
    particles.clear();
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},
        {1,0,1},{0,1,1},{1,0.5,0},{1,0.5,0.5},{0.7, 1, 0.5},{0.5,0.7,0},
        {0,0.5,1},{0,1,0.5},{0,0.5,0},{0,0,0},{0.5,0,0}};
    float diameter_ball = 2.0f * radius_ball;
    float offset_z = std::sqrt(3.0f) * radius_ball; // Found by looking for the z that minimizes the norm between 2 balls
    vec3 offset_triangle = vec3{-2.0f * diameter_ball, 0.03f, -.8f};

    size_t index = 0;
    for (size_t i = 0; i < 5; ++i) {
        for (size_t j = 0; j < 5 - i; ++j) {
            particle_structure new_ball;
            new_ball.r = radius_ball;
            new_ball.c = color_lut[index];
            new_ball.p = vec3(diameter_ball * (j + i / 2.0f), 0.01f,  i * offset_z) + offset_triangle;
            particles.push_back(new_ball);
            index++;
        }
    }
}

void scene_model::display_particles(scene_structure& scene) {
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
    float height_border = 0.05f;
    float size_hole = 0.12f;
    float corner_hole_bias = 0.1f;
    float width_hole_side = 0.01f;
    float below = -0.5f;

    // A pool table is symmetric along its z and x axis so we don't need to specify all corners
    std::vector<aabb> corners = {{x_ground - width_hole_side, x_ground, below, height_border, 0, size_hole / 2.0f, false},
                                 {x_ground - width_border, x_ground - width_hole_side, below, height_border, size_hole / 2.0f, size_hole / 2.0f, false},
                                 {x_ground - width_border, x_ground, below, height_border, size_hole / 2.0f, z_ground - width_border - corner_hole_bias, true},
                                 {x_ground - width_hole_side, x_ground, below, height_border, z_ground - width_border - corner_hole_bias, z_ground - width_hole_side, false},
                                 {x_ground - width_border - corner_hole_bias, x_ground - width_hole_side, below, height_border, z_ground - width_hole_side, z_ground, false},
                                 {0, x_ground - width_border - corner_hole_bias, below, height_border, z_ground - width_border, z_ground, true}};

    for (int symmetric_x = -1; symmetric_x <= 1; symmetric_x += 2) {
        for (int symmetric_z = -1; symmetric_z <= 1; symmetric_z += 2) {
            for (const auto& corner : corners) {
                boundaries.emplace_back(corner.minX * symmetric_x, corner.maxX * symmetric_x, corner.minY, corner.maxY, corner.minZ * symmetric_z, corner.maxZ * symmetric_z, corner.bouncing);
            }
        }
    }

    boundaries.emplace_back(-x_ground + width_border, x_ground - width_border, -0.2, 0, -z_ground + width_border, z_ground - width_border, true);

    ground = mesh_drawable(mesh_primitive_quad({-x_ground + width_border,0,-z_ground + width_border}, {x_ground - width_border,0,-z_ground + width_border}, {x_ground - width_border,0,z_ground - width_border}, {-x_ground + width_border,0,z_ground - width_border}));

    // This line must be called after boundaries is full
    borders = segments_gpu(segments_from_aabbs((boundaries)));
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];
}

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& ) {
    glfwWindowHint(GLFW_SAMPLES, 4);
    glEnable(GL_MULTISAMPLE);
    textRenderer.setup_font(shaders);
    setup_aabb(shaders);
    triangle_base_configuration();
    white_ball_setup();
    scene.camera.camera_type = camera_control_spherical_coordinates;
    animationCamera.update_camera(scene, -white_ball_position, vec2(0,0));

    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    this->pool = mesh_load_file_obj("scenes/animation/Pool_game/assets/pool.obj");
    this->plane_pool = mesh_load_file_obj("scenes/animation/Pool_game/assets/plane_pool.obj");
    texture_wood  = create_texture_gpu(image_load_png("scenes/animation/Pool_game/assets/wood.png"));
    texture_green  = create_texture_gpu(image_load_png("scenes/animation/Pool_game/assets/green.png"));
    texture_cue = create_texture_gpu(image_load_png("scenes/animation/Pool_game/assets/macassar.png"));

    pool_visual = mesh_drawable(pool, shaders["mesh_bf"], texture_wood);
    pool_visual.uniform.shading.specular = 0.0f;

    plane_pool_visual = mesh_drawable(plane_pool, shaders["mesh_bf"], texture_green);
    plane_pool_visual.uniform.shading.specular = 0.0f;
}

void scene_model::reset_game() {
    triangle_base_configuration();
    white_ball_setup();
    nb_shots = 0;
    score = 0;
    gameFinished = false;
}

void scene_model::set_gui() {
    // Can set the speed of the animation
    if (ImGui::Checkbox("Reset Pool Board", &reset)) {
        reset = false;
        reset_game();
    }
    ImGui::Checkbox("Animate camera", &animateCamera);
    ImGui::Checkbox("The point click is the direction of the shot", &pointDir);
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Camera animation time duration", &animationCamera.animation_duration, 0.2f, 5.0f, "%.2f s");
    ImGui::SliderInt("Number of collisions computations in one frame", &steps_in_frame, 1, 20);
    ImGui::SliderFloat("Max launch speed", &max_speed, 1.0f, 15.f, "%.1f");
    ImGui::SliderFloat("Friction", &alpha, .97f, .99f, "%.3f s");
    ImGui::SliderFloat("Impact", &beta, 0.97f, .99f, "%.3f s");
    ImGui::SliderInt("Score", (int*)&score, 0, 100);
    ImGui::SliderInt("Number of shots", (int*)&nb_shots, 0, 100);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}

aabb::aabb(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, bool bouncing) {
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
    this->bouncing = bouncing;
}

bool intersectPlane(const vec3 &n, const vec3 &p0, const ray r, vec3& p) {
    // assuming vectors are all normalized
    float denom = dot(n, r.u);

    if (denom > 1e-6) {
        vec3 p0l0 = p0 - r.p;
        float t = dot(p0l0, n) / denom;

        if (t >= 0) {
            p = r.p + r.u * t;
            return true;
        }
    }

    return false;
}

void scene_model::mouse_click(scene_structure& scene, GLFWwindow* window, int , int , int ) {
    // Mouse click is used to select a position of the control polygon
    // ******************************************************************** //

    // Cursor coordinates
    const vec2 cursor = glfw_cursor_coordinates_window(window);

    // Check that the mouse is clicked (drag and drop)
    const bool mouse_click_left = glfw_mouse_pressed_left(window);
    const bool mouse_released_left = glfw_mouse_released_left(window);

    // Check if shift key is pressed
    if (mouse_click_left && play_allowed && !in_animation) {
        // Create the 3D ray passing by the selected point on the screen
        const ray r = picking_ray(scene.camera, cursor);

        // Check if this ray intersects the ground
        vec3 p0 = vec3(0, 0, 0);
        vec3 n = vec3(0, -1, 0);

        vec3 p;

        if (intersectPlane(n, p0, r, p)) {
            throw_pos = p;
            is_throwing = true;
            if (pointDir)
                cane_direction = normalize(p - particles[0].p); // The cane is in the direction of the shoot
        }
    }

    else if (mouse_released_left && play_allowed && is_throwing && !in_animation) {
        is_throwing = false;

        if (distance == 0)
            return;

        play_allowed = false;
        ++nb_shots;

        particles[0].v = throw_dir * distance * 10.f;

        float norm_speed = norm(particles[0].v);
        if (norm_speed > max_speed) {
            particles[0].v = (particles[0].v / norm_speed) * max_speed;
        }

        particles[0].v.y = 0;
        distance = 0;
    }
}

void scene_model::mouse_move(scene_structure& scene, GLFWwindow* window) {
    // Cursor coordinates
    const vec2 cursor = glfw_cursor_coordinates_window(window);

    // Create the 3D ray passing by the selected point on the screen
    const ray r = picking_ray(scene.camera, cursor);

    // Check if this ray intersects the ground
    vec3 p0 = vec3(0, 0, 0);
    vec3 n = vec3(0, -1, 0);

    vec3 p;

    if (intersectPlane(n, p0, r, p)) {
        if (is_throwing) {
            distance = dot(-throw_dir, p - throw_pos);
            distance = std::max(0.f, distance);
        }
        else if (play_allowed){
            if (pointDir)
                throw_dir = normalize(p - particles[0].p);
            else
                throw_dir = normalize(particles[0].p - scene.camera.camera_position());
        }
    }
    cane_direction = normalize(particles[0].p - scene.camera.camera_position()); // The cane is in the direction of the camera
}

void scene_model::check_score() {
    for (size_t i = 1; i < particles.size(); ++i)
    {
        if (particles[i].p.y <= -2.0f) {
            particles.erase(particles.begin() + i);
            i--;
            score++;
        }
    }
}

void scene_model::check_white_ball() {
    if (particles[0].p.y <= -2.0f) {
        particles[0].p = white_ball_position;
        particles[0].v = vec3(0, 0, 0);
        ++nb_shots; // If we pocket the white ball, we have a penalty
    }
}

void scene_model::check_end_game() {
    if (particles.size() == 1) { // We pocketed all the balls expect the white one
        gameFinished = true;
    }
}

bool scene_model::check_state() {
    for (const auto& particle: particles) {
        if (fabs(particle.v.x) > 1e-2 || fabs(particle.v.z) > 1e-2) {
            return false;
        }
    }

    return true;
}

static float linear_interpolation(float t, float t_final, const float angle1, const float angle2) {
    const float alpha = (t)/(t_final); //percentage of advancement in the current line, from 0 to 1
    float angle = (1-alpha)*angle1 + alpha*angle2;

    return angle;
}

static vec3 linear_interpolation(float t, float t_final, const vec3 p1, const vec3 p2) {
    const float alpha = (t)/(t_final); //percentage of advancement in the current line, from 0 to 1
    vec3 p = (1-alpha)*p1 + alpha*p2;

    return p;
}

float animation_camera::angle_between_center_and_point(const vec3& point) {
    // Trigonometry magic using the dot product to find the angle we need
    vec2 toward_center = vec2(-point.x, -point.z);
    vec2 z = vec2(0, -1);
    float angle = std::acos(dot(toward_center, z) / (norm(toward_center) * norm(z)));
    if (point.x < 0)
        angle = -angle; // Need to inverse the angle if we are on one half of the board
    return angle;
}

void animation_camera::update_camera(scene_structure& scene, vec3 position, vec2 spherical_coordinates) {
    scene.camera.translation = position;//-particles[0].p;
    scene.camera.spherical_coordinates.x = spherical_coordinates.x;//-0.45; // In radians
    scene.camera.spherical_coordinates.y = spherical_coordinates.y;;

    scene.camera.apply_rotation(0,0,0,0); // To make our changes to spherical coordinates update
}

void scene_model::draw_cue(std::map<std::string,GLuint>& shaders, scene_structure& scene) {
    if (!play_allowed)
        return;

    vec3 cane_dir_tmp = cane_direction;
    cane_dir_tmp.y = -0.2f;
    cane_dir_tmp = normalize(cane_dir_tmp);
    // We convert to spherical coordinate to be able to specify an angle
    // and still keeping the normalized vector
    float phi = std::atan2(cane_dir_tmp.z, cane_dir_tmp.x);
    cane_dir_tmp.x = std::cos(phi) * std::sin(M_PI / 4.0f);
    cane_dir_tmp.z = std::sin(phi) * std::sin(M_PI / 4.0f);
    float angle = 10.0f; // Angle in degree for the cue
    cane_dir_tmp.y = -std::cos( (90.0f - angle) * M_PI / 180.0f);

    vec3 p1 = particles[0].p - cane_dir_tmp * .05 - cane_dir_tmp * (distance / 4);
    vec3 p2 = p1 - cane_dir_tmp * .75;

    mesh_drawable pool_cue = mesh_primitive_cylinder(0.02f, p1, p2);
    pool_cue.shader = shaders["mesh_bf"];
    pool_cue.texture_id = texture_cue;

    draw(pool_cue, scene.camera);
}

#endif

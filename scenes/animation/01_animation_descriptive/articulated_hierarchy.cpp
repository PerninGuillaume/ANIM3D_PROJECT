
#include "articulated_hierarchy.hpp"


#ifdef SCENE_ARTICULATED_HIERARCHY


using namespace vcl;



void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    const float radius_body = 0.25f;
    const float radius_head = 0.15f;
    const float radius_eye = 0.03f;
    const float radius_arm = 0.05f;
    const float length_arm = 0.2f;
    const float scaling_axis_z = 1.7f;

    // The geometry of the body is a sphere
    mesh_drawable body = mesh_drawable( mesh_primitive_sphere(radius_body, {0,0,0}, 40, 40));
    body.uniform.transform.scaling_axis = {1.0,1.0, scaling_axis_z};

    // Geometry of the beak: cone

    mesh_drawable beak = mesh_drawable(mesh_primitive_cone(radius_head / 2.2f, {0,0,0}, {0,0,0.2f}));
    beak.uniform.color = vec3{255,164,46} / 255.0f;
    // Geometry of the eyes: black spheres

    mesh_drawable head = mesh_drawable(mesh_primitive_sphere(radius_head, {0,0,0}, 40, 40));
    mesh_drawable eye = mesh_drawable(mesh_primitive_sphere(radius_eye, {0,0,0}, 20, 20));
    eye.uniform.color = {0,0,0};

    // Shoulder part and arm are displayed as cylinder
    mesh_drawable shoulder = mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});
    mesh_drawable arm = mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});

    // An elbow displayed as a sphere
    mesh_drawable elbow = mesh_primitive_sphere(0.055f);
    //Biceps
    mesh_drawable biceps = mesh_primitive_sphere(0.055f);
    //Hand
    mesh_drawable hand = mesh_primitive_sphere(0.055f);

    // Build the hierarchy:
    // Syntax to add element
    //   hierarchy.add(visual_element, element_name, parent_name, (opt)[translation, rotation])
    hierarchy.add(body, "body");

    hierarchy.add(head, "head", "body", radius_body * vec3(0.0f, 9/10.0f, scaling_axis_z * 9/10.0f));

    hierarchy.add(beak, "beak", "head", radius_head * vec3(0.0f, 0.0f, 0.9f));
    // Eyes positions are set with respect to some ratio of the
    hierarchy.add(eye, "eye_left", "head" , radius_head * vec3( 1/3.0f, 1/2.0f, 1/1.5f));
    hierarchy.add(eye, "eye_right", "head", radius_head * vec3(-1/3.0f, 1/2.0f, 1/1.5f));

    // Set the left part of the body arm: shoulder-elbow-arm
    hierarchy.add(shoulder, "shoulder_left", "body", {-radius_body+0.05f,0,0}); // extremity of the spherical body
    hierarchy.add(elbow, "elbow_left", "shoulder_left", {-length_arm,0,0});     // place the elbow the extremity of the "shoulder cylinder"
    hierarchy.add(arm, "arm_bottom_left", "elbow_left");                        // the arm start at the center of the elbow
    hierarchy.add(hand, "hand_left", "arm_bottom_left", {-length_arm, 0, 0});
    hierarchy.add(biceps, "biceps_left", "shoulder_left", {-length_arm / 2.0f, 0, 0});

    // Set the right part of the body arm: similar to the left part excepted a symmetry is applied along x direction for the shoulder
    hierarchy.add(shoulder, "shoulder_right", "body",     {{radius_body-0.05f,0,0}, {-1,0,0, 0,1,0, 0,0,1}/*Symmetry*/ } );
    hierarchy.add(elbow, "elbow_right", "shoulder_right", {-length_arm,0,0});
    hierarchy.add(arm, "arm_bottom_right", "elbow_right");
    hierarchy.add(hand, "hand_right", "arm_bottom_right", {-length_arm, 0, 0});
    hierarchy.add(biceps, "biceps_right", "shoulder_right", {-length_arm / 2.0f, 0.01f, 0});


  // Set the same shader for all the elements
    hierarchy.set_shader_for_all_elements(shaders["mesh"]);



    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);

    timer.scale = 0.5f;
}




void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui();

    // Current time
    const float t = timer.t;

    /** *************************************************************  **/
    /** Compute the (animated) transformations applied to the elements **/
    /** *************************************************************  **/


    // The body oscillate along the z direction
    //hierarchy["body"].transform.translation = {0,0,0.2f*(1+std::sin(2*3.14f*t))};

    // The head nods
    mat3 const R_head = rotation_from_axis_angle_mat3({1,0,0}, 0.3f * std::sin(2 * 3.14 * t));

    // Rotation of the shoulder around the y axis
    mat3 const R_shoulder = rotation_from_axis_angle_mat3({0,0,1}, std::sin(2*3.14f*(t)) );
    // Rotation of the arm around the y axis (delayed with respect to the shoulder)
    mat3 const R_arm = rotation_from_axis_angle_mat3({0,0,1}, std::sin(2*3.14f*(t)) );
    // Symmetry in the x-direction between the left/right parts
    mat3 const Symmetry = {-1,0,0, 0,1,0, 0,0,1};

    // Set the rotation to the elements in the hierarchy
    hierarchy["head"].transform.rotation = R_head;
    hierarchy["shoulder_left"].transform.rotation = R_shoulder;
    hierarchy["arm_bottom_left"].transform.rotation = R_arm;
    float growing = 1.0f + 1.5f * std::sin(3.14 * 2.0f * (t - 0.5f));
    float growing_color = std::sin(3.14 * 2.0f * (t - 0.5f));
    float decading_color = 1.0f - sin((t - 0.5f) * 3.14 * 2.0f);
    if (t > 0.5f) {
      hierarchy["biceps_left"].transform.scaling = growing;
      hierarchy["head"].element.uniform.color = {1.0f, decading_color, decading_color};
    }
    else {
      hierarchy["head"].element.uniform.color = {1, 1, 1};
    }


    hierarchy["shoulder_right"].transform.rotation = Symmetry*R_shoulder; // apply the symmetry
    hierarchy["arm_bottom_right"].transform.rotation = R_arm; //note that the symmetry is already applied by the parent element
    if (t > 0.5f)
      hierarchy["biceps_right"].transform.scaling = growing;

  hierarchy.update_local_to_global_coordinates();


    /** ********************* **/
    /** Display the hierarchy **/
    /** ********************* **/

    if(gui_scene.surface) // The default display
        draw(hierarchy, scene.camera);

    if(gui_scene.wireframe) // Display the hierarchy as wireframe
        draw(hierarchy, scene.camera, shaders["wireframe"]);

    if(gui_scene.skeleton) // Display the skeleton of the hierarchy (debug)
        hierarchy_visual_debug.draw(hierarchy, scene.camera);

}


void scene_model::set_gui()
{
    ImGui::Text("Display: "); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe); ImGui::SameLine();
    ImGui::Checkbox("Surface", &gui_scene.surface);     ImGui::SameLine();
    ImGui::Checkbox("Skeleton", &gui_scene.skeleton);   ImGui::SameLine();

    ImGui::Spacing();
    ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max);
    ImGui::SliderFloat("Time scale", &timer.scale, 0.01f, 3.0f);

}





#endif


#ifndef OWDS_BULLETCLIENT_H
#define OWDS_BULLETCLIENT_H

#include "SharedMemory/PhysicsClientC_API.h"

#include <string>
#include <array>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <mutex>

#include <ros/ros.h>

namespace owds {

enum BulletShapeType_e {
    GEOM_SPHERE = 2,
	GEOM_BOX,
	GEOM_CYLINDER,
	GEOM_MESH,
	GEOM_PLANE,
	GEOM_CAPSULE,  //non-standard URDF?
	GEOM_SDF,      //signed-distance-field, non-standard URDF
	GEOM_HEIGHTFIELD,
	GEOM_UNKNOWN
};

enum Renderer_e {
    TINY_RENDERER = (1) << (16),
    BULLET_HARDWARE_OPENGL = (1) << (17)
};

enum RendererFlags_e {
	ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX = 1,
	ER_USE_PROJECTIVE_TEXTURE = 2,
	ER_NO_SEGMENTATION_MASK = 4,
};

enum UrdfFlags_e
{
	URDF_USE_INERTIA_FROM_FILE = 2,  //sync with URDFJointTypes.h 'ConvertURDFFlags'
	URDF_USE_SELF_COLLISION = 8,     //see CUF_USE_SELF_COLLISION
	URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
	URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
	URDF_RESERVED = 64,
	URDF_USE_IMPLICIT_CYLINDER = 128,
	URDF_GLOBAL_VELOCITIES_MB = 256,
	MJCF_COLORS_FROM_FILE = 512,
	URDF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
	URDF_ENABLE_SLEEPING = 2048,
	URDF_INITIALIZE_SAT_FEATURES = 4096,
	URDF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,
	URDF_PARSE_SENSORS = 16384,
	URDF_USE_MATERIAL_COLORS_FROM_MTL = 32768,
	URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,
	URDF_MAINTAIN_LINK_ORDER = 131072,
	URDF_ENABLE_WAKEUP = 1 << 18,
	URDF_MERGE_FIXED_LINKS = 1 << 19,
	URDF_IGNORE_VISUAL_SHAPES = 1 << 20,
	URDF_IGNORE_COLLISION_SHAPES = 1 << 21,
	URDF_PRINT_URDF_INFO = 1 << 22,
	URDF_GOOGLEY_UNDEFINED_COLORS = 1 << 23,
};

struct aabb_t
{
    std::array<double, 3> min;
    std::array<double, 3> max;
    bool is_valid;

    aabb_t() : min{0}, max{0}, is_valid{false}
    {}
};

class BulletClient
{
public:
    BulletClient(b3PhysicsClientHandle* client_handle, size_t client_id);
    ~BulletClient();

    size_t getId() { return client_id_; }

    void setAdditionalSearchPath(const std::string& path);
    void configureDebugVisualizer(b3ConfigureDebugVisualizerEnum flag, bool enable);

    int createVisualShapeBox(const std::array<double, 3>& half_extents, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeSphere(float radius, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeCylinder(float radius, float height, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeCapsule(float radius, float height, const std::array<double, 4>& rgba_color = {1});
    int createVisualShapeMesh(const std::string& file_name, const std::array<double, 3>& scale, const std::array<double, 4>& rgba_color = {1,1,1,1});

    struct b3VisualShapeInformation getVisualShapeData(int object_id, int flags = 0);

    int createCollisionShapeBox(const std::array<double, 3>& half_extents, int flags = 0);
    int createCollisionShapeSphere(float radius, int flags = 0);
    int createCollisionShapeCylinder(float radius, float height, int flags = 0);
    int createCollisionShapeCapsule(float radius, float height, int flags = 0);
    int createCollisionShapeMesh(const std::string& file_name, const std::array<double, 3>& scale, int flags = 0);

    int createMultiBody(float base_mass,
                        int base_collision_shape_index,
                        int base_visual_shape_index,
                        const std::array<double, 3>& base_position,
                        const std::array<double, 4>& base_orientation,
                        int flags = 0);

    int loadTexture(const std::string& file_path);
    bool changeTexture(int object_id, int joint_index, int texture_id);
    bool changeRgbaColor(int object_id, int joint_index, const std::array<double, 4>& color);
    bool changeSpecularColor(int object_id, int joint_index, const std::array<double, 3>& color);

    int loadURDF(const std::string& file_name,
                const std::array<double, 3>& base_position,
                const std::array<double, 4>& base_orientation,
                bool use_fixed_base = false,
                int flags = 0);

    int loadURDFRaw(const std::string& raw_urdf, const std::string& temp_file_name,
                           const std::array<double, 3>& base_position,
                           const std::array<double, 4>& base_orientation,
                           bool use_fixed_base = false,
                           int flags = 0);

    int getNumJoints(int body_id);
    bool resetJointState(int body_id, int joint_index, double target_value, double target_velocity = 0);
    void resetBaseVelocity(int body_id, const std::array<double, 3>& linear_velocity, const std::array<double, 3>& angular_velocity);
    void resetBasePositionAndOrientation(int body_id, const std::array<double, 3>& position, const std::array<double, 4>& orientation);
    std::pair<std::array<double, 3>, std::array<double, 4>> getBasePositionAndOrientation(int body_id);
    long createUserConstraint(int parent_body_id, int parent_link_index,
                              int child_body_id, int child_link_index,
                              JointType joint_type,
                              const std::array<double, 3>& joint_axis,
                              const std::array<double, 3>& parent_frame_pose,
                              const std::array<double, 3>& child_frame_pose,
                              const std::array<double, 4>& parent_frame_orientation,
                              const std::array<double, 4>& child_frame_orientation);
    void removeUserConstraint(int user_constraint_id);
    void changeUserConstraint(int user_constraint_id,
                             const std::array<double, 3>& joint_child_pivot,
                             const std::array<double, 4>& joint_child_frame_orientation,
                             double max_force = -1);

    struct b3LinkState getLinkState(int body_id, int link_index, bool compute_link_velocity = false, bool compute_forward_kinematics = false);
    struct b3JointInfo getJointInfo(int body_id, int joint_index);
    std::pair<std::unordered_map<std::string, int>, std::unordered_map<std::string, int>> findJointAndLinkIndices(int body_id);

    void setMass(int body_id, int link_index, double mass_kg);
    void setLateralFriction(int body_id, int link_index, double friction);
    void setSpinningFriction(int body_id, int link_index, double friction);
    void setRollingFriction(int body_id, int link_index, double friction);
    void setRestitution(int body_id, int link_index, double restitution);
    void setFrictionAnchor(int body_id, int link_index, double friction);
    void setActivationState(int body_id, DynamicsActivationState state);

    std::array<float, 16> computeProjectionMatrix(float fov,
                                                  float aspect,
                                                  float near_value,
                                                  float far_value);

    std::array<float, 16> computeProjectionMatrix(float left,
                                                  float right,
                                                  float bottom,
                                                  float top,
                                                  float near_value,
                                                  float far_value);

    std::array<float, 16> computeProjectionMatrix(const std::array<float, 3>& camera_position,
                                                 float distance,
                                                 float yaw,
                                                 float pitch,
                                                 float roll,
                                                 int up_axis_index);
                                                
    std::array<float, 16> computeViewMatrix(const std::array<float, 3>& camera_eye_position,
                                            const std::array<float, 3>& camera_target_position,
                                            const std::array<float, 3>& camera_up_vector);

    std::array<float, 16> computeViewMatrix(const std::array<float, 3>& camera_target_position,
                                            float distance,
                                            float yaw,
                                            float pitch,
                                            float roll,
                                            int up_axis_index);

    struct b3CameraImageData getCameraImage(int width, int height,
                                            std::array<float, 16>& view_matrix, 
                                            std::array<float, 16>& projection_matrix,
                                            Renderer_e renderer,
                                            int flags = -1);

    std::unordered_set<int> getSegmentationIds(const b3CameraImageData& image);

    long addUserDebugLine(const std::array<double, 3>& xyz_from,
                          const std::array<double, 3>& xyz_to,
                          const std::array<double, 3>& color_rgb,
                          double line_width = 1,
                          double life_time = 0,
                          int replace_id = -1,
                          int parent_object_id = -1,
                          int parent_link_index = -1);

    long addUserDebugPoint(const std::array<double, 3>& xyz,
                          const std::array<double, 3>& color_rgb,
                          double point_size = 0.2,
                          double life_time = 0,
                          int pointNum = 1,
                          int replace_id = -1,
                          int parent_object_id = -1,
                          int parent_link_index = -1);

    long addUserDebugText(const std::string& text,
                          const std::array<double, 3>& position,
                          const std::array<double, 3>& color_rgb,
                          float text_size = 0.1,
                          float life_time = 0,
                          long parent_object_id = -1,
                          int replace_id = -1);

    bool removeUserDebugItem(int unique_id);

    bool removeAllUserDebugItem();

    bool removeCollisionShape(int unique_id);

    std::vector<struct b3RayHitInfo> rayTestBatch(const std::vector<std::array<double, 3>>& from_poses,
                                                  const std::vector<std::array<double,3>>& to_poses,
                                                  int nb_thread = 1,
                                                  bool report_hit_number = false);
    
    void performCollisionDetection();

    struct aabb_t getAABB(int body_id, int link_index = -1);
    struct b3AABBOverlapData getOverlappingObjects(const struct aabb_t& aabb);
    struct b3ContactInformation getContactPoints(int body_id_A, int body_id_B = -1, int link_index_A = -2, int link_index_B = -2);

    void resetDebugVisualizerCamera(float distance, float yaw, float pitch, const std::array<float,3>& target_pose);
    b3MouseEventsData getMouseEvents();
    b3KeyboardEventsData getKeyboardEvents();

    void setGravity(double grivity_x, double grivity_y, double grivity_z);
    void setTimeStep(double time_step);
    void stepSimulation();
    
private:
    b3PhysicsClientHandle* client_handle_;
    size_t client_id_;
    std::mutex mutex_;

    std::string additional_path_;
    std::unordered_map<size_t, int> loaded_textures_;
    std::unordered_map<size_t, int> loaded_collision_meshes_;
    // Visual meshes are not cached as same texture would be 
    // applied to all same visual meshes

    int createVisualShape(BulletShapeType_e shape_type, 
                            float radius,
                            const std::array<double, 3>& half_extents,
                            float height,
                            const std::string& file_name, 
                            const std::array<double, 3>& mesh_scale,
                            const std::array<double, 4>& rgba_color);

    int createCollisionShape(BulletShapeType_e shape_type, 
                            float radius,
                            const std::array<double, 3>& half_extents,
                            float height,
                            const std::string& file_name, 
                            const std::array<double, 3>& mesh_scale,
                            int flags);
};

} // namespace owds

#endif // OWDS_BULLETCLIENT_H
////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni & Sahit Chintalapudi
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include <footstep_planner/graphs/nav_lattice_8D.h>
#include <footstep_planner/proto/robot_parameters/unpack_from_proto.h>
#include <footstep_planner/rviz/visualize.h>
#include <footstep_planner/utils/state_conversions.h>
#include <cassert>
#include <cmath>
#include <iostream>

namespace footstep_planner {
namespace graphs {

namespace utils = utils::state_conversions;

NavLattice8D::NavLattice8D(
    const std::shared_ptr<smpl::SparseDistanceMap> distance_map,
    const proto::RobotParameters& robot_details,
    const std::shared_ptr<graphs::HomotopyInformation> homotopy_information,
    const std::shared_ptr<environment::proto::ValidSteppingCells>
        valid_stepping_cells)
    : distance_map_(distance_map),
      homotopy_information_(homotopy_information),
      valid_stepping_cells_(valid_stepping_cells)
{
    footstep_planner::proto::unpack_from_proto(&robot_parameters_,
                                               robot_details);
    bipedal_ID_to_state_.clear();
    treeContainer.bipedal_ID_to_state_ = &NOTDOM;
    kdtree = std::make_shared<kdtree_>(
        4 /*dim*/, treeContainer,
        nanoflann::KDTreeSingleIndexAdaptorParams(10));
}

NavLattice8D::~NavLattice8D()
{
    // Free Foot States
    for (size_t i = 0; i < foot_ID_to_state_.size(); ++i) {
        delete (foot_ID_to_state_[i]);
    }
    foot_ID_to_state_.clear();

    // Free Bipedal States
    for (size_t i = 0; i < bipedal_ID_to_state_.size(); ++i) {
        delete (bipedal_ID_to_state_[i]);
    }
    bipedal_ID_to_state_.clear();

    goal_state_ = nullptr;
}

int NavLattice8D::set_bipedal_state(const double& x, const double& y,
                                    const double& z, const double& theta_rad)
{
    const double nominal_offset_m = robot_parameters_.nominal_offset_m;
    const double res = distance_map_->resolution();

    /*
     *const double right_x_direction = nearbyint(cos(theta_rad));
     *const double right_y_direction = nearbyint(sin(theta_rad));
     */
    const double right_x_direction = cos(theta_rad);
    const double right_y_direction = sin(theta_rad);

    const Eigen::Vector4d right_foot(
        x + (nominal_offset_m / 2.0) * right_x_direction,
        y + (nominal_offset_m / 2.0) * right_y_direction, z, theta_rad);

    const double offset_theta_rad =
        utils::normalize_angle_rad(theta_rad + M_PI);
    /*
     *const double left_x_direction = nearbyint(cos(offset_theta_rad));
     *const double left_y_direction = nearbyint(sin(offset_theta_rad));
     */
    const double left_x_direction = cos(offset_theta_rad);
    const double left_y_direction = sin(offset_theta_rad);

    const Eigen::Vector4d left_foot(
        x + (nominal_offset_m / 2.0) * left_x_direction,
        y + (nominal_offset_m / 2.0) * left_y_direction, z, theta_rad);

    FootState* left_foot_state = get_foot_state(left_foot);
    if (left_foot_state == nullptr) {
        left_foot_state = create_new_foot_state(left_foot);
        if (nullptr == left_foot_state) {
            ROS_ERROR("[NavLattice8D] Invalid left foot state.");
            return -1;
        }
    }

    FootState* right_foot_state = get_foot_state(right_foot);
    if (right_foot_state == nullptr) {
        right_foot_state = create_new_foot_state(right_foot);
        if (nullptr == right_foot_state) {
            ROS_ERROR("[NavLattice8D] Invalid right foot state.");
            return -1;
        }
    }

    BipedalState* new_state = new BipedalState();
    new_state->next_foot = right;
    new_state->left_foot_id = left_foot_state->id;
    new_state->right_foot_id = right_foot_state->id;
    new_state->set_center(x, y, z, theta_rad);
    new_state->pid = -10;   // assuming start&goal state have no parent.

    int bipedal_id = create_new_bipedal_state(new_state);
    // const auto map_elem = bipedal_state_to_ID_.find(new_state);
    // if (map_elem == bipedal_state_to_ID_.end()) {
    //} else {
    // bipedal_id = map_elem->second;
    //}

    if (!is_valid_bipedal_state(bipedal_id)) {
        return -1;
    }

    return bipedal_id;

}  // set_bipedal_state

int NavLattice8D::set_start_state(const double& x_, const double& y_,
                                  const double& z_, const double& theta_rad)
{
    // const int theta = utils::continuous_angle_to_discrete(
    // theta_rad,
    // robot_parameters_.num_theta_vals);
    const double res = distance_map_->resolution();
    double x = x_ * res;
    double y = y_ * res;
    double z = z_ * res;

    ROS_INFO("[NavLattice8D] Start state: (%.3f, %.3f, %.3f, %.3f)", x, y, z,
             theta_rad);

    /*
     *const int rows = distance_map_->numCellsY();
     *const int cols = distance_map_->numCellsX();
     *ROS_INFO("Dimension of the environment: %d %d.", rows, cols);
     */

    FootState* start_state = get_foot_state(x, y, z, theta_rad);
    if (start_state == nullptr) {
        start_state = create_new_foot_state(x, y, z, theta_rad);
    }

    const int bipedal_id = set_bipedal_state(x, y, z, theta_rad);
    if (bipedal_id == -1) {
        ROS_ERROR("[NavLattice8D] Invalid Bipedal Start State.");
    }

    return bipedal_id;
}

int NavLattice8D::set_goal_state(const double& x_, const double& y_,
                                 const double& z_, const double& theta_rad)
{
    // const int theta = utils::continuous_angle_to_discrete(
    // theta_rad,
    // robot_parameters_.num_theta_vals);
    const double res = distance_map_->resolution();
    double x = x_ * res;
    double y = y_ * res;
    double z = z_ * res;

    ROS_INFO("[NavLattice8D] Goal state: (%f, %f, %f, %f)", x, y, z, theta_rad);

    goal_state_ = get_foot_state(x, y, z, theta_rad);
    if (goal_state_ == nullptr) {
        goal_state_ = create_new_foot_state(x, y, z, theta_rad);
    }

    const int bipedal_id = set_bipedal_state(x, y, z, theta_rad);
    if (bipedal_id == -1) {
        ROS_ERROR("[NavLattice8D] Invalid Bipedal Goal State.");
    }

    return bipedal_id;
}

bool NavLattice8D::is_goal(const int& current_state_id)
{
    const BipedalState* bipedal_state = get_bipedal_state(current_state_id);
    if (bipedal_state == nullptr) return false;

    const Eigen::Vector4d center_feet_pos = bipedal_state->get_center();

    const Eigen::Vector4d goal_pos =
        get_continuous_coordinates(goal_state_->id);
    const double dist_to_goal =
        (center_feet_pos.head<3>() - goal_pos.head<3>()).norm();

    if (dist_to_goal <= robot_parameters_.goal_tolerance_m) {
        return true;
    }

    return false;
}

std::size_t NavLattice8D::hashkey_4d(double x_, double y_, double z_,
                                     double theta_) const
{
    std::size_t seed = 7;
    std::size_t x, y, z, theta;
    x = boost::hash_value(x_);
    y = boost::hash_value(y_);
    z = boost::hash_value(z_);
    theta = boost::hash_value(theta_);

    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    boost::hash_combine(seed, z);
    boost::hash_combine(seed, theta);

    return seed;
}

int NavLattice8D::get_foot_state_cellid(const int& x, const int& y,
                                        const int& z, const int& theta) const
{
    const int cols = distance_map_->numCellsX();
    const int rows = distance_map_->numCellsY();
    const int height = distance_map_->numCellsZ();
    return (x + (cols * (y + rows * (z + (height * theta)))));
}

int NavLattice8D::get_foot_state_id(const double& x, const double& y,
                                    const double& z,
                                    const double& theta_rad) const
{
    auto key = hashkey_4d(x, y, z, theta_rad);
    auto state = foot_state_to_ID_.find(key);
    if (state == foot_state_to_ID_.end()) {
        // ROS_ERROR("[NAVLattice8D] Foot state does not exist in hash table.");
        return -1;
    }

    return (state->second)->id;
}

Eigen::Vector4d NavLattice8D::get_continuous_coordinates(
    const int& state_4D_id) const
{
    Eigen::Vector4d cont_coordinates(0, 0, 0, 0);

    auto state = get_foot_state(state_4D_id);

    if (!state) {
        ROS_ERROR("[NavLattice8D] Invalid state ID.(get_cont_coord)");
        return cont_coordinates;
    }

    cont_coordinates =
        Eigen::Vector4d(state->x, state->y, state->z, state->theta);
    return cont_coordinates;
}

bool NavLattice8D::is_valid_bipedal_state(const int& state_id)
{
    std::vector<Eigen::Vector4d> transformed_spheres;
    get_transformed_collision_spheres(state_id, &transformed_spheres);

    for (const auto sphere : transformed_spheres) {
        if (distance_map_->getDistance(sphere.x(), sphere.y(), sphere.z()) <=
            sphere.w()) {
            return false;
        }
    }

    const BipedalState* bipedal_state = get_bipedal_state(state_id);
    if (bipedal_state == nullptr) return false;

    const FootState* left_foot_state =
        get_foot_state(bipedal_state->left_foot_id);
    if (left_foot_state == nullptr) {
        ROS_ERROR("[NavLattice8D] Left foot 4D state does not exist.");
        return false;
    }

    const FootState* right_foot_state =
        get_foot_state(bipedal_state->right_foot_id);
    if (right_foot_state == nullptr) {
        ROS_ERROR("[NavLattice8D] Right foot 4D state does not exist.");
        return false;
    }

    return true;
}

bool NavLattice8D::is_valid_foot_state(const Eigen::Vector3d& foot_pos)
{
    return is_valid_foot_state(foot_pos.x(), foot_pos.y(), foot_pos.z());
}

bool NavLattice8D::is_valid_foot_state(const double& x, const double& y,
                                       const double& z)
{
    int x_m;
    int y_m;
    int z_m;

    distance_map_->worldToGrid(x, y, z, x_m, y_m, z_m);

    if (distance_map_->getDistance(x_m, y_m, z_m) <= 0) {
        return false;
    }
    const int rows = distance_map_->numCellsY();
    const int cols = distance_map_->numCellsX();
    const int index_3d = cols * (rows * z_m + y_m) + x_m;

    return valid_stepping_cells_->stepping_cell(index_3d);
}

void NavLattice8D::get_transformed_collision_spheres(
    const int& bipedal_state_id,
    std::vector<Eigen::Vector4d>* transformed_spheres)
{
    const BipedalState* bipedal_state = get_bipedal_state(bipedal_state_id);
    if (bipedal_state == nullptr) return;

    const Eigen::Vector4d left_foot_pos =
        get_continuous_coordinates(bipedal_state->left_foot_id);

    const Eigen::Vector4d right_foot_pos =
        get_continuous_coordinates(bipedal_state->right_foot_id);

    const Eigen::Vector4d center_feet_pos = bipedal_state->get_center();

    // Check the collision spheres do not intersect anything in the environment
    for (int i = 0; i < robot_parameters_.collision_spheres.size(); i++) {
        Eigen::Vector4d origin_pos = center_feet_pos;
        const auto origin = robot_parameters_.collision_spheres[i].origin;
        if (origin == robot_details::Origin::LEFT_FOOT) {
            origin_pos = left_foot_pos;
        }
        else if (origin == robot_details::Origin::RIGHT_FOOT) {
            origin_pos = right_foot_pos;
        }

        const Eigen::Vector2d sphere =
            Eigen::Translation2d(origin_pos.x(), origin_pos.y()) *
            Eigen::Rotation2Dd(origin_pos.w()) *
            Eigen::Vector2d(robot_parameters_.collision_spheres[i].x,
                            robot_parameters_.collision_spheres[i].y);

        const double sphere_z_m =
            origin_pos.z() + robot_parameters_.collision_spheres[i].z;

        Eigen::Vector4d transformed_sphere(
            sphere.x(), sphere.y(), sphere_z_m,
            robot_parameters_.collision_spheres[i].radius);

        transformed_spheres->push_back(transformed_sphere);
    }
}  // get_transformed_collision_spheres

FootState* NavLattice8D::get_foot_state(const Eigen::Vector4d& state_pos) const
{
    return get_foot_state(state_pos.x(), state_pos.y(), state_pos.z(),
                          state_pos.w());
}

FootState* NavLattice8D::get_foot_state(const double& x, const double& y,
                                        const double& z,
                                        const double& theta_rad) const
{
    // check if cell is in distance_map bounds
    int x_m, y_m, z_m;
    distance_map_->worldToGrid(x, y, z, x_m, y_m, z_m);
    if (!distance_map_->isCellValid(x_m, y_m, z_m)) return nullptr;

    const int state_id = get_foot_state_id(x, y, z, theta_rad);
    if (state_id == -1) return nullptr;

    return get_foot_state(state_id);
}

FootState* NavLattice8D::get_foot_state(const int& state_id) const
{
    if (state_id >= foot_ID_to_state_.size()) {
        ROS_ERROR("[NavLattice8D] Invalid state ID(get_foot_state).");
        return nullptr;
    }
    return foot_ID_to_state_.at(state_id);
}

BipedalState* NavLattice8D::get_bipedal_state(const int& state_id)
{
    if (state_id >= bipedal_ID_to_state_.size()) {
        ROS_ERROR("[NavLattice8D] Bipedal State does not exist.");
        return nullptr;
    }

    return bipedal_ID_to_state_[state_id];
}

FootState* NavLattice8D::create_new_foot_state(const Eigen::Vector4d& state_pos)
{
    return create_new_foot_state(state_pos.x(), state_pos.y(), state_pos.z(),
                                 state_pos.w());
}

FootState* NavLattice8D::create_new_foot_state(const double& x, const double& y,
                                               const double& z,
                                               const double& theta_rad)
{
    if (is_valid_foot_state(x, y, z)) {
        int id = foot_ID_to_state_.size();
        FootState* ptr = new FootState(id, x, y, z, theta_rad);
        foot_state_to_ID_[hashkey_4d(x, y, z, theta_rad)] = ptr;
        // ROS_INFO("hashkey for (%.3f %.3f %.3f %.3f) is: %lu", x, y,
        // z,theta_rad,  hashkey_4d(x,y,z,theta_rad));
        foot_ID_to_state_.push_back(ptr);
        return ptr;
    }
    return nullptr;
}

int NavLattice8D::create_new_bipedal_state(BipedalState* new_state)
{
    const int state_id = bipedal_ID_to_state_.size();
    new_state->id = state_id;
    new_state->domID = -1;  // default
    bipedal_ID_to_state_.push_back(new_state);
    bipedal_state_to_ID_[hashkey_4d(new_state->x, new_state->y, new_state->z,
                                    new_state->theta)] = new_state;


    return state_id;
}

double NavLattice8D::get_action_cost(const FootState* source_state,
                                     const FootState* new_state)
{
    const Eigen::Vector3d source_pos(source_state->x, source_state->y,
                                     source_state->z);
    const Eigen::Vector3d new_pos(new_state->x, new_state->y, new_state->z);
    const double norm_val = ((new_pos - source_pos).norm());
    return (norm_val > 0.0001) ? norm_val * 100.0 : 100.0;
}

int NavLattice8D::get_active_foot_id(const BipedalState& bipedal_state)
{
    if (bipedal_state.next_foot == left) {
        return bipedal_state.left_foot_id;
    }
    return bipedal_state.right_foot_id;
}

int NavLattice8D::get_pivot_foot_id(const BipedalState& bipedal_state)
{
    if (bipedal_state.next_foot == left) {
        return bipedal_state.right_foot_id;
    }
    return bipedal_state.left_foot_id;
}

void NavLattice8D::get_foot_succs(const BipedalState& bipedal_state,
                                  std::vector<int>* foot_succs)
{
    // Get the pivot foot
    const int pivot_foot_id = get_pivot_foot_id(bipedal_state);
    Eigen::Vector4d pivot_foot = get_continuous_coordinates(pivot_foot_id);

    // If we're pivoting around the right foot, the direction for the
    // pivot point must be offset by 180 degrees
    const Foot pivot_foot_type = bipedal_state.next_foot == left ? right : left;
    const double pivot_direction =
        pivot_foot_type == right
            ? utils::normalize_angle_rad(pivot_foot.w() + M_PI)
            : pivot_foot.w();

    const double foot_rot_direction = pivot_foot_type == right ? 1.0 : -1.0;

    // Get the active foot
    const int active_foot_id = get_active_foot_id(bipedal_state);
    const Eigen::Vector4d active_foot =
        get_continuous_coordinates(active_foot_id);

    const double eps = 0.00001;
    // Get the pivot point with respect to the active foot (i.e. the foot you
    // want to move).
    const double nominal_offset_m = robot_parameters_.nominal_offset_m;
    const double x_pivot_direction =
        fabs(cos(pivot_direction)) <= eps
            ? 0.0
            : (cos(pivot_direction) < 0 ? -1.0 : 1.0);
    const double y_pivot_direction =
        fabs(sin(pivot_direction)) <= eps
            ? 0.0
            : (sin(pivot_direction) < 0 ? -1.0 : 1.0);
    const Eigen::Vector4d pivot_point(
        pivot_foot.x() + nominal_offset_m * x_pivot_direction,
        pivot_foot.y() + nominal_offset_m * y_pivot_direction, active_foot.z(),
        pivot_foot.w());

    // Forward direction of feet is offset by 90 degrees
    const double direction =
        utils::normalize_angle_rad(pivot_foot.w() + M_PI / 2);

    const double res = distance_map_->resolution();
    const double x_direction =
        fabs(cos(direction)) <= eps ? 0.0 : (cos(direction) < 0 ? -1.0 : 1.0);
    const double y_direction =
        fabs(sin(direction)) <= eps ? 0.0 : (sin(direction) < 0 ? -1.0 : 1.0);

    for (const auto mprim : robot_parameters_.motion_primitives) {
        const Eigen::Vector4d successor(
            pivot_point.x() + mprim.x() * x_direction * res,
            pivot_point.y() + mprim.y() * y_direction * res,
            pivot_point.z() + mprim.z() * res,
            pivot_point.w() + mprim.w() * foot_rot_direction);

        if (!is_valid_foot_state(successor.head<3>())) {
            continue;
        }
        // Ignore no motion
        if ((successor - active_foot).norm() <= eps) {
            continue;
        }

        FootState* new_foot_state = get_foot_state(successor);
        if (new_foot_state == nullptr) {
            new_foot_state = create_new_foot_state(successor);
        }

        foot_succs->push_back(new_foot_state->id);
    }
}  // get_foot_succs

void NavLattice8D::get_succs(const int& bipedal_state_id,
                             std::vector<int>* succ_ids,
                             std::vector<double>* costs)
{
    const BipedalState* bipedal_state = get_bipedal_state(bipedal_state_id);
    if (bipedal_state == nullptr) return;

    const int pivot_foot_id = get_pivot_foot_id(*bipedal_state);
    const int active_foot_id = get_active_foot_id(*bipedal_state);

    std::vector<int> foot_succs;
    get_foot_succs(*bipedal_state, &foot_succs);

    for (const int foot_succ_id : foot_succs) {
        // const Eigen::Vector4d averaged_state =
        // get_cont_averaged_state(pivot_foot_id, foot_succ_id);

        // const Eigen::Vector4d parent_avg_state = bipedal_state->get_center();

        BipedalState* new_state = new BipedalState();
        new_state->next_foot =
            (bipedal_state->next_foot == left) ? right : left;
        new_state->left_foot_id =
            (bipedal_state->next_foot == left) ? foot_succ_id : pivot_foot_id;
        new_state->right_foot_id =
            (bipedal_state->next_foot == right) ? foot_succ_id : pivot_foot_id;

        const Eigen::Vector4d bpd_center = get_cont_averaged_state(
            new_state->left_foot_id, new_state->right_foot_id);
        new_state->set_center(bpd_center);

        int bipedal_id;
        auto key = hashkey_4d(new_state->x, new_state->y, 
                new_state->z, new_state->theta);
        const auto map_elem = bipedal_state_to_ID_.find(key);
        if (map_elem == bipedal_state_to_ID_.end()) {
            new_state->pid = bipedal_state_id;
            bipedal_id = create_new_bipedal_state(new_state);
            // int  bipedal_id = create_new_bipedal_state(new_state);
        }
        else {
            bipedal_id = (map_elem->second)->id;
        }

        // Skip invalid bipedal states
        if (!is_valid_bipedal_state(bipedal_id)) {
            continue;
        }

        const FootState* active_foot = get_foot_state(active_foot_id);
        const FootState* foot_succ = get_foot_state(foot_succ_id);
        const double cost = get_action_cost(active_foot, foot_succ);

        succ_ids->push_back(bipedal_id);
        costs->push_back(cost);
    }
}  // get_succs

Eigen::Vector4d NavLattice8D::get_cont_averaged_state(
    const int& left_state_id, const int& right_state_id) const
{
    const Eigen::Vector4d left_foot_cont =
        get_continuous_coordinates(left_state_id);
    const Eigen::Vector4d right_foot_cont =
        get_continuous_coordinates(right_state_id);

    Eigen::Vector4d avg_state = (left_foot_cont + right_foot_cont) * 0.5;

    const Eigen::Vector2d vec_left(cos(left_foot_cont.w()),
                                   sin(left_foot_cont.w()));

    const Eigen::Vector2d vec_right(cos(right_foot_cont.w()),
                                    sin(right_foot_cont.w()));

    const Eigen::Vector2d vec_center =
        (vec_left != vec_right) ? vec_left + vec_right : vec_left;

    avg_state.w() =
        utils::normalize_angle_rad(atan2(vec_center.y(), vec_center.x()));

    return avg_state;
}

Eigen::Vector4i NavLattice8D::get_disc_averaged_state(
    const int& left_state_id, const int& right_state_id) const
{
    const Eigen::Vector4d cont_state =
        get_cont_averaged_state(left_state_id, right_state_id);

    int x, y, z;
    distance_map_->worldToGrid(cont_state.x(), cont_state.y(), cont_state.z(),
                               x, y, z);

    const int theta = utils::continuous_angle_to_discrete(
        cont_state.w(), robot_parameters_.num_theta_vals);

    const auto avg_state = Eigen::Vector4i(x, y, z, theta);

    return avg_state;
}

double NavLattice8D::GetInflation() {
    return 3.0;
}

int NavLattice8D::GetNearestNeighbor(int stateID) {
    if(stateID >= (int)bipedal_ID_to_state_.size() ){
        ROS_ERROR("ERROR in NavLattice8D... function: GetNearestNeighbor stateID illegal.");
    }

    BipedalState* qstate = bipedal_ID_to_state_.at(stateID);
    double res = distance_map_->resolution(); 
    double radius = 0.141*res;      // the min step size is 0.141 cellsize
    
    /* nanoflann */
    double query_pt[4] = {qstate->x, qstate->y, qstate->z, qstate->theta};
    const std::size_t numofneighbors = 40;      // depends on number of nearest neighbors
    std::size_t neibIndex[numofneighbors];
    double distance[numofneighbors];

    nanoflann::KNNResultSet<double> resultSet(numofneighbors);
    resultSet.init(neibIndex, distance);
    auto found = kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(0));

    if (found) {
        for (auto i = 0; i < numofneighbors; ++i) {
            
            auto neighbor = NOTDOM.at(neibIndex[i]);
            if (neighbor->pid == qstate->pid || neighbor->id == qstate->pid) {
                continue;
            } else {
                double dist = std::sqrt(distance[i]);
                if (dist > radius ) return -1;
                return neibIndex[i];
            }
        }
    } 

    return -1;

}   // GetNearestNeighbor


void NavLattice8D::InsertIntoDOM(int stateID) {
    if(stateID >= (int)bipedal_ID_to_state_.size() ){
        ROS_ERROR("ERROR in NavLattice8D... function: InsertIntoDom stateID illegal.");
    }

    // get the state
    auto state = bipedal_ID_to_state_.at(stateID);
    state->domID = NOTDOM.size();

    // insert into container
    NOTDOM.push_back(state);
    kdtree->addPoints( (size_t)NOTDOM.size()-1, (size_t)NOTDOM.size()-1);
}

void NavLattice8D::RemoveFromDOM(int stateID){ 
    if(stateID >= (int)bipedal_ID_to_state_.size() ){
        ROS_ERROR("ERROR in NavLattice8D... function: RemoveFromDOM stateID illegal.");
    }

    // get the state
    auto state = bipedal_ID_to_state_.at(stateID);
    if (-1 == state->domID) return;

    kdtree->removePoint(state->domID);
    NOTDOM.erase(NOTDOM.begin() + state->domID);
    state->domID = -1;
}


}  // namespace graphs
}  // namespace footstep_planner


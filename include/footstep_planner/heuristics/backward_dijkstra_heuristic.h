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

#ifndef SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_HEURISTICS_BACKWARD_DIJKSTRA_HEURISTIC_H_
#define SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_HEURISTICS_BACKWARD_DIJKSTRA_HEURISTIC_H_

#include <footstep_planner/planners/dijkstra.h>
#include <environment_projections.pb.h>

namespace footstep_planner {
namespace heuristics {

// This class implements a backward 2D Dijkstra heuristic
class BackwardDijkstraHeuristic {
 public:
    // The class takes a dijkstra planner and the projected environment
    BackwardDijkstraHeuristic(
        const std::shared_ptr<planners::Dijkstra> dijkstra_planner,
        const std::shared_ptr<environment::proto::EnvironmentProjections>
            env_projections) : \
        dijkstra_planner_(dijkstra_planner),
        env_projections_(env_projections) {}

    // Returns the backward dijkstra heuristic value for the give state
    int get_heuristic_value(
        const int& id,
        const int& x,
        const int& y,
        const int& z);

 private:
    const std::shared_ptr<planners::Dijkstra> dijkstra_planner_;
    const std::shared_ptr<environment::proto::EnvironmentProjections>
        env_projections_;
};

}  // namespace heuristics
}  // namespace footstep_planner

#endif  // SRC_FOOTSTEP_PLANNER_INCLUDE_FOOTSTEP_PLANNER_HEURISTICS_BACKWARD_DIJKSTRA_HEURISTIC_H_

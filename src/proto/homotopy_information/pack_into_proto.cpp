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

#include <footstep_planner/proto/homotopy_information/pack_into_proto.h>

namespace footstep_planner {
namespace graphs {
namespace proto {

Beams_Beam pack_into_proto(
    const int& x,
    const int& y,
    const int& workspace,
    const int& letter) {
    Beams_Beam beam;
    beam.set_x(x);
    beam.set_y(y);
    beam.set_workspace(workspace);
    beam.set_letter(letter);

    return beam;
}

void pack_into_proto(
    Beams* beams_proto,
    const std::vector<Beams_Beam>& beams) {
    for (const auto beam : beams) {
        beams_proto->add_beams()->CopyFrom(beam);
    }
}

Gates_Gate pack_into_proto(
    const int& lower_workspace_idx,
    const int& upper_workspace_idx,
    const int& letter) {
    environment::proto::WorkspaceIndices workspace_indices;
    workspace_indices.set_lower_workspace_idx(lower_workspace_idx);
    workspace_indices.set_upper_workspace_idx(upper_workspace_idx);

    Gates_Gate gate;
    gate.mutable_workspaces()->CopyFrom(workspace_indices);
    gate.set_letter(letter);

    return gate;
}

void pack_into_proto(
    Gates* gates_proto,
    const std::vector<Gates_Gate>& gates) {
    for (const auto gate : gates) {
        gates_proto->add_gates()->CopyFrom(gate);
    }
}

}  // namespace proto
}  // namespace graphs
}  // namespace footstep_planner

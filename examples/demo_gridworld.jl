using LinearAlgebra
using StaticArrays: SA
using DomainSets: Disk
using HybridAStar: hybrid_astar_search

dynamics(state, action) = state + SA[cosd(action), sind(action)]

node_key(state) = round.(Int, state)

function node_g(old_state, new_state, action, depth)
    if new_state in Disk(2.0, SA[6.0, 6.0])
        return Inf
    end
    return 1.0
end

node_h(state) = norm(state - SA[6.0, 6.0])

start_state = SA[2.0, 2.0]
actions = [0, 30, 60, 90, 120, 150, 180] # angles that the vehicle can travel
goal = Disk(1.0, SA[9.0,9.0])
cs = hybrid_astar_search(goal, start_state, actions, dynamics, node_key, node_g, node_h)
p = get_path(start_state, cs, dynamics)

#=
p = get_path(start_state, cs, dynamics)
x = [s[1] for s in p]; y = [s[2] for s in p]
plot(x,y)

using JET
using BenchmarkTools
@report_opt hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
@profiler hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
@benchmark hybrid_astar_search($g, $holonomic_vs, $holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
=#

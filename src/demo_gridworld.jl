using LinearAlgebra

# Base.in(state::Tuple{Int64, Int64}, goal::Array{Tuple{Int64, Int64}, 1}) = state in goal
# Base.in(state, goal) = state in goal

#Dynamics
dynamics(state, action) = state .+ action

#NodeKey
node_key(state) = state

#Node Cost
function node_cost(old_state,new_state,action,depth)
    #Cost from going out of bounds
    if(new_state[1]>10 || new_state[2]>10 || new_state[1]<0 || new_state[2]<0)
        return Inf
    end

    if( new_state in [ (3,5),(7,3) ] )
        return Inf
    end

    return 1.0
end

#Heuristic Cost
heuristic_cost(state) = sqrt( (state[1]-9)^2 + (state[2]-9)^2 )


start_state = (2,2)
actions = [ (1,0), (-1,0), (0,1), (0,-1) ]
g = [(9,9)]
cs = hybrid_astar_search(g, start_state, actions, dynamics, node_key, node_cost, heuristic_cost)


#=
using JET
using BenchmarkTools
@report_opt hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
@profiler hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
@benchmark hybrid_astar_search($g, $holonomic_vs, $holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
=#

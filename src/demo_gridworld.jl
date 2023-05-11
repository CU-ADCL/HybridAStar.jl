using LinearAlgebra
using StaticArrays


struct GWEnvironment
    length::Float64
    breadth::Float64
    obstacles::Array{Tuple{Int64, Int64},1}
end

function Base.in(agent_state::Tuple{Int64, Int64}, goal::Tuple{Int64, Int64})
    if( agent_state[1] == goal[1] && agent_state[2] == goal[2])
         return true
    else
        return false
    end
end

#Dynamics
function agent_dynamics(agent_state::Tuple{Int64, Int64},a::Tuple{Int64, Int64})
    new_agent_state = agent_state .+ a
    return new_agent_state
end

#NodeKey
function get_node_key(agent_state::Tuple{Int64, Int64})
    return agent_state
end

#Node Cost
struct node_cost <: Function
    env::GWEnvironment
end

function (obj::node_cost)(old_agent_state,new_agent_state,action,depth)

    total_cost = 0.0
    world = obj.env

    #Cost from going out of bounds
    if(new_agent_state[1]>world.length || new_agent_state[2]>world.breadth || new_agent_state[1]<0 || new_agent_state[2]<0)
        return Inf
    end

    #Cost from collision with obstacles
    for obstacle in world.obstacles
        if( in(new_agent_state,obstacle) )
            return Inf
        end
    end

    #Cost from Long Paths
    total_cost += 1

    return total_cost
end

#Heuristic Cost
struct heuristic_cost <: Function
    goal::Tuple{Int64, Int64}
end

function (obj::heuristic_cost)(agent_state)
    euclidean_distance =  sqrt( (agent_state[1] - obj.goal[1])^2 + (agent_state[2] - obj.goal[2])^2 )
    return euclidean_distance
end

function main()
    e = GWEnvironment(20,20,[(10,10),(15,5)])
    agent_start = (2,2)
    agent_actions = [ (1,0), (-1,0), (0,1), (0,-1) ]
    g = (17,18)
    nc = node_cost(e)
    hc = heuristic_cost(g)
    cs = hybrid_astar_search(g, agent_start, agent_actions, agent_dynamics, get_node_key, nc, hc)
    return cs
end


#=
using JET
using BenchmarkTools
@report_opt hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
@profiler hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
@benchmark hybrid_astar_search($g, $holonomic_vs, $holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
=#

function get_action_sequence(current_node::GraphNode, nodes_dict)
    action_sequence = typeof(current_node.parent_action)[]
    while(current_node.parent_key!==nothing)
        push!(action_sequence, current_node.parent_action)
        current_node = nodes_dict[current_node.parent_key]
    end
    return reverse(action_sequence)
end


"""
    get_path(start_state, action_sequence, dynamics)

The function returns the path followed by the agent from a given starting state under a given dynamics function and given sequence of actions.

# Arguments

- `start_state` -> specifies the agent's starting state
- `action_sequence` -> array of agent actions to be applied in sequence
- `dynamics` -> function describing the dynamics of the agent. It takes in the state and an action as inputs and outputs the next resulting state.
```julia-repl
        next_state = dynamics(state,action)
```

# Output

- array of agent states from its start state to the final state after applying all the actions in the action_sequence

# Example
```julia-repl
agent_path = get_path(start_state, action_sequence, dynamics)
```

"""
function get_path(start_state, action_sequence, dynamics)
    path = Array{typeof(start_state),1}([start_state])
    current_state = start_state
    for action in action_sequence
        new_state = dynamics(current_state,action)
        push!(path, new_state)
        current_state = new_state
    end
    return path
end


"""
    hybrid_astar_search(goal, start_state, actions, dynamics, discretization, g, h; planning_time=0.2, lambda=1.0)

The function returns a sequence of actions that can be applied to the agent to take it from its starting position to its goal position.

# Arguments

- `goal::Any`: specifies the agent's goal location/set; Any object that supports `in(goal, state)` can be used.
- `start_state::Any`: specifies the agent's starting state
- `actions::AbstractVector`: list of available agent actions
- `dynamics::Function`: function describing the dynamics of the agent. It takes in the state and an action as inputs and outputs the next resulting state. It will be called like this: `next_state = dynamics(state, action)`
- `discretization::Function`: function that is used to discretize the continuous agent state. It takes in the state as an input and outputs the corresponding discrete value. It will be called like this: `discrete_key = discretization(state)`
- `g::Function`: function that calculates the cost of transitioning from one state to the other. It takes in the old state, the new state, the applied action and the depth of the new state in the graph as inputs and outputs the corresponding cost. To penalize collision with obstacles in the environment, user should reflect it in this function. `g` will be called like this: `cost = g(old_state, new_state, action, new_depth)`
- `h::Function`: function that calculates the heuristic cost of reaching the goal from any given state. It takes in the state as an input and outputs the corresponding heuristic cost. It will be called like this: `heuristic_cost = h(state)`

# Keyword Arguments

- `planning_time::Real=0.2`: specifies the planning time in seconds allocated to find a feasible path
- `lambda::Real=0.99`: specifies the discounting factor to weigh initial steps of the agent's trajectory more than the later steps

# Output

- [if path found] array of agent actions that can guide the agent from its start state to its goal region
- [if path not found] empty array

# Example
```jldoctest
using LinearAlgebra
using StaticArrays: SA
using DomainSets: Disk
using HybridAStar: hybrid_astar_search,get_path

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
```
"""
function hybrid_astar_search(goal, agent_state, agent_actions, agent_dynamics, node_key, node_cost, heuristic_cost; planning_time=0.2, lambda=0.99)

    # Initialize variables
    depth = 0       #Variable to keep track of path length
    k = node_key(agent_state)
    key_type = typeof(k)
    state_type = typeof(agent_state)
    action_type = typeof(agent_actions[1])
    open = PriorityQueue{key_type,Float64}(Base.Order.Forward)
    closed = Set{key_type}()
    nodes_dict = Dict{key_type,GraphNode{state_type,action_type,key_type}}()
    path = typeof(agent_actions)()

    h = heuristic_cost(agent_state)
    open[k] = h
    start_node = GraphNode{state_type,action_type,key_type}(agent_state,k,0.0,h,nothing,nothing,depth)
    nodes_dict[k] = start_node

    start_time = time()
    while(length(open) != 0)
        current_node = nodes_dict[dequeue!(open)]
        #Add the popped node to the closed list, because there is no better way to reach this node.
        push!(closed, current_node.key)

        if( in(current_node.state,goal) )
            # println("Time taken to find the Hybrid A* path: ", time()- start_time)
            return get_action_sequence(current_node, nodes_dict)
        end

        current_agent_state = current_node.state
        new_depth = current_node.depth+1

        for action in agent_actions
            new_agent_state = agent_dynamics(current_agent_state,action)
            new_node_key = node_key(new_agent_state)

            if( new_node_key in closed )
                continue
            else
                #Calculate actual cost of the new node
                g = current_node.actual_cost + (lambda^new_depth)*node_cost(current_agent_state,new_agent_state,action,new_depth)
                #Calculate heuristic cost of the new node
                h = heuristic_cost(new_agent_state)
                #Define new graph node
                new_node = GraphNode{state_type,action_type,key_type}(new_agent_state,new_node_key,g,h,current_node.key,action,new_depth)

                if(g == Inf)
                    # closed[new_node_key] = new_node
                    push!(closed, new_node_key)
                    continue
                end
                if(haskey(open,new_node_key))
                    if(nodes_dict[new_node_key].actual_cost > new_node.actual_cost)
                        nodes_dict[new_node_key] = new_node
                        open[new_node_key] = new_node.heuristic_cost + new_node.actual_cost
                    end
                else
                    nodes_dict[new_node_key] = new_node
                    open[new_node_key] = new_node.heuristic_cost + new_node.actual_cost
                end
            end
        end
        if(time()- start_time >= planning_time)
            println("Planning time exceeded for Hybrid A* planning")
            return path
        end
    end
    println("Path not found")
    return path
end

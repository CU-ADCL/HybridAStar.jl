function get_path(current_node::GraphNode, nodes_dict)
    controls_sequence = typeof(current_node.parent_action)[]
    while(current_node.parent_key!=nothing)
        push!(controls_sequence, current_node.parent_action)
        current_node = nodes_dict[current_node.parent_key]
    end
    return reverse(controls_sequence)
end

function hybrid_astar_search(goal, agent_state, agent_actions, agent_dynamics, node_key, node_cost, heuristic_cost; planning_time=0.2, λ=0.99)

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

    h = heuristic_cost(goal, agent_state)
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
            return get_path(current_node, nodes_dict)
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
                g = current_node.actual_cost + (λ^new_depth)*node_cost(goal,new_agent_state,action,new_depth)
                #Calculate heuristic cost of the new node
                h = heuristic_cost(goal, new_agent_state)
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

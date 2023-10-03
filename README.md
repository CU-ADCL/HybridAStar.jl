# HybridAStar

[![Build Status](https://github.com/himanshugupta1009/HybridAStar.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/himanshugupta1009/HybridAStar.jl/actions/workflows/CI.yml?query=branch%3Amain)

This is a Julia package for using the Hybrid A* algorithm. Hybrid A* algorithm is typically used to find a drivable path for a Dubin's car in an obstacle rich environment. However, it can be used for other problems as well that fall in the same category! (Problems with continuous/discrete state space and finite action space)    

## Information on Hybrid A* algorithm can be found in these papers

- Junior: The Stanford entry in the Urban Challenge [(Link)](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.20258)
- Practical Search Techniques in Path Planning for Autonomous Driving [(Link)](https://ai.unist.ac.kr/~chiu/robot/papers/dolgov_gpp_stair08.pdf)
- Application of Hybrid A\* to an Autonomous Mobile Robot for Path Planning in Unstructured Outdoor Environments [(Link)](https://ieeexplore.ieee.org/abstract/document/6309512)

## This package provides users with the following functions:

### 1) `hybrid_astar_search`

    hybrid_astar_search(goal, start_state, actions, dynamics, discretization, g, h; planning_time=0.2, lambda=1.0)

The function returns a sequence of actions that can be applied to the agent to take it from its starting position to its goal position.

#### Arguments

- `goal::Any`: specifies the agent's goal location/set; Any object that supports `in(goal, state)` can be used.
- `start_state::Any`: specifies the agent's starting state
- `actions::AbstractVector`: list of available agent actions
- `dynamics::Function`: function describing the dynamics of the agent. It takes in the state and an action as inputs and outputs the next resulting state. It will be called like this: `next_state = dynamics(state, action)`
- `discretization::Function`: function that is used to discretize the continuous agent state. It takes in the state as an input and outputs the corresponding discrete value. It will be called like this: `discrete_key = discretization(state)`
- `g::Function`: function that calculates the cost of transitioning from one state to the other. It takes in the old state, the new state, the applied action and the depth of the new state in the graph as inputs and outputs the corresponding cost. To penalize collision with obstacles in the environment, user should reflect it in this function. `g` will be called like this: `cost = g(old_state, new_state, action, new_depth)`
- `h::Function`: function that calculates the heuristic cost of reaching the goal from any given state. It takes in the state as an input and outputs the corresponding heuristic cost. It will be called like this: `heuristic_cost = h(state)`

#### Keyword Arguments

- `planning_time::Real=0.2`: specifies the planning time in seconds allocated to find a feasible path
- `lambda::Real=0.99`: specifies the discounting factor to weigh initial steps of the agent's trajectory more than the later steps

#### Output

- [if path found] array of agent actions that can guide the agent from its start state to its goal region
- [if path not found] empty array

### 2) `get_path`

    get_path(start_state, action_sequence, dynamics)

The function returns the path followed by the agent from a given starting state under a given dynamics function and given sequence of actions.
    
  #### Arguments
  
  - `start_state::Any`: specifies the agent's starting state. 
  - `action_sequence::AbstractVector`: array of agent actions to be applied in sequence 
  - `dynamics::Function`: function describing the dynamics of the agent. It takes in the state and an action as inputs and outputs the next resulting state. 
     It will be called like this: `next_state = dynamics(state,action)`

#### Output
  - array of agent states from its start state to the final state after applying all the actions in the action_sequence  


***


#### Examples

1) A simple example to find path for a holonomic vehicle using the Hybrid A* algorithm can be found in examples/demo_holonomic_vehicle.jl

2) A more complex example of finding a drivable path for a Dubin's car using the Hybrid A* algorithm can be found in examples/demo_dubin_car.jl


In case of any queries, feel free to raise a Github Issue or reach out to the author via email at himanshu.gupta@colorado.edu.

  

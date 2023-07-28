# HybridAStar.jl

This is a Julia package for using the Hybrid A* algorithm. Hybrid A* algorithm is typically used to find a drivable path for a Dubin's car in an obstacle rich environment. However, it can be used for other problems as well that fall in the same category! (Problems with continuous/discrete state space and finite action space)    

## Information on Hybrid A* algorithm can be found in these papers

* Junior: The Stanford entry in the Urban Challenge [(Link)](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.20258)
* sdsd [(Link)](https://ai.unist.ac.kr/~chiu/robot/papers/dolgov_gpp_stair08.pdf)
* Application of Hybrid A* to an Autonomous Mobile Robot for Path Planning in Unstructured Outdoor Environments [(Link)](https://ieeexplore.ieee.org/abstract/document/6309512)

## This package provides users with the following functions:

### 1) hybrid_astar_search
    returns a sequence of actions that can be applied to the agent to take it from its starting position to its goal position. 

Function parameters:
  * *goal* -> specifies the agent's goal location/set.
  * *start_state* -> specifies the agent's starting state. 
  * *actions* -> an array of available agent actions 
  * *dynamics* -> function describing the dynamics of the agent. It takes in the state and an action as inputs and outputs the next resulting state. 
    * ```next_state = dynamics(state,action)```
  * *discretization* -> function that is used to discretize the continuous agent state to a discrete bin. It takes in the state as an input and outputs the corresponding discrete value.
    * ```discrete_key = discretization(state)```      
  * *g* -> function that calculates the cost of transitioning from one state to the other. It takes in the old_state, the new_state, the applied_action and the corresponding depth of the new_state in the graph as inputs and outputs the corresponding cost.
    * To penalize collision with obstacles in the environment, user should reflect it in this function. 
    * ```cost = g(old_state, new_state, action, new_depth)```  
  * *h* -> function that calculates the heuristic cost of reaching the goal from any given state. It takes in the state as an input and outputs the corresponding heuristic cost.
    * ```heuristic_cost = h(state)```   
  * planning_time (optional; default_value=0.2) -> specifies the planning time allocated to find a feasible path
  * lambda (optional; default_value=1.0) -> specifies the discounting factor to weigh initial steps of the agent's trajectory more than the later steps. 

  Output: 
  * [if path found] array of agent actions that can guide the agent from its start state to its goal region (if path found)
  * [if path not found] empty array

  Usage: 
  ```
  action_sequence = hybrid_astar_search(goal, start_state, actions, dynamics, discretization, g, h; planning_time=1.0, λ=0.95)
  ```

### 2) get_path
    returns the path followed by the agent from a given starting state under a given dynamics function and given sequence of actions.
    
  Function parameters:
  * *start_state* -> specifies the agent's starting state. 
  * *action_sequence* -> array of agent actions to be applied in sequence 
  * dynamics -> function describing the dynamics of the agent. It takes in the state and an action as inputs and outputs the next resulting state. 
    * ```next_state = dynamics(state,action)```

  Output: 
  * array of agent states from its start state to the final state after applying all the actions in the action_sequence  

  Usage:

  ```
  agent_path = get_path(start_state, action_sequence, dynamics)
  ```


### A simple example to find path for a holonomic vehicle using the Hybrid A* algorithm can be found in examples/demo_holonomic_vehicle.jl

### A more complex example of finding a drivable path for a Dubin's car using the Hybrid A* algorithm can be found in examples/demo_dubin_car.jl


In case of any queries, feel free to raise a Github Issue or reach out to the author via email at himanshu.gupta@colorado.edu.

  

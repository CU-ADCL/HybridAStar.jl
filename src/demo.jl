using LinearAlgebra
using StaticArrays

struct Location
    x::Float64
    y::Float64
end

struct ObstacleLocation
    x::Float64
    y::Float64
    r::Float64 #Radius of the obstacle which is assumed to be a circle
end

struct ExperimentEnvironment
    length::Float64
    breadth::Float64
    obstacles::Array{ObstacleLocation,1}
end


#State
struct VehicleState <: FieldVector{3,Float64}
    x::Float64
    y::Float64
    theta::Float64
end

function Base.in(agent_state::VehicleState, goal::Location)
    euclidean_distance = sqrt( (agent_state.x - goal.x)^2 + (agent_state.y - goal.y)^2 )
    if( euclidean_distance < 2.0)
         return true
    else
        return false
    end
end

#Actions
function get_vehicle_actions(max_delta_angle, min_delta_angle_difference)
    set_of_delta_angles = Float64[0.0]
    half_num_actions = Int( floor(max_delta_angle/min_delta_angle_difference) )
    for i in 1:half_num_actions
        neg_angle = -min_delta_angle_difference*i*pi/180
        pos_angle = min_delta_angle_difference*i*pi/180
        if(wrap_between_0_and_2Pi(neg_angle) != wrap_between_0_and_2Pi(pos_angle))
            push!(set_of_delta_angles, neg_angle)
            push!(set_of_delta_angles, pos_angle)
        else
            push!(set_of_delta_angles, pos_angle)
        end
    end
    return set_of_delta_angles
end

function wrap_between_0_and_2Pi(theta)
   return mod(theta,2*pi)
end


#Dynamics
function holonomic_vehicle_dynamics(vehicle_state::VehicleState,delta_angle)
    vehicle_speed = 1.0
    time_duration = 0.5
    vehicle_x = vehicle_state.x
    vehicle_y = vehicle_state.y
    vehicle_theta = vehicle_state.theta
    if(vehicle_speed == 0.0)
        return VehicleState(vehicle_x,vehicle_y,vehicle_theta,vehicle_speed)
    else
        new_theta = wrap_between_0_and_2Pi(vehicle_theta + delta_angle)
        new_x = vehicle_x + vehicle_speed*cos(new_theta)*time_duration
        new_y = vehicle_y + vehicle_speed*sin(new_theta)*time_duration
        return VehicleState(new_x,new_y,new_theta)
    end
end

#NodeKey
struct NodeBin <: FieldVector{3,Float64}
    discrete_x::Float64
    discrete_y::Float64
    discrete_θ::Float64
end

function get_node_key(vehicle_state::VehicleState)
    world_length = 100.0
    world_breadth = 100.0
    bin_width = 0.25
    max_num_bins_x = ceil(world_length/bin_width)
    discrete_x = clamp(ceil(vehicle_state.x/bin_width),1.0,max_num_bins_x)
    max_num_bins_y = ceil(world_breadth/bin_width)
    discrete_y = clamp(ceil(vehicle_state.y/bin_width),1.0,max_num_bins_y)
    discrete_theta = ceil(vehicle_state.theta*180/pi)
    return NodeBin(discrete_x,discrete_y,discrete_theta)
end

#Node Cost
struct node_cost <: Function
    wheelbase::Float64
    env::ExperimentEnvironment
end

function (obj::node_cost)(goal,vehicle_state,action,time_stamp)

    total_cost = 0.0
    vehicle_x = vehicle_state.x
    vehicle_y = vehicle_state.y
    vehicle_L = obj.wheelbase
    world = obj.env

    #Cost from going out of bounds
    if(vehicle_x>world.length-vehicle_L || vehicle_y>world.breadth-vehicle_L || vehicle_x<0.0+vehicle_L || vehicle_y<0.0+vehicle_L)
        return Inf
    end

    #Cost from collision with obstacles
    for obstacle in world.obstacles
        if(in_obstacle(vehicle_x,vehicle_y,obstacle,vehicle_L))
            return Inf
        else
            continue
        end
    end

    #Cost from no change in heading angle
    if(action == 0.0)
       total_cost += -1.0
    end

    #Cost from Long Paths
    total_cost += 1

    return total_cost
end

function is_within_range(p1_x,p1_y, p2_x, p2_y, threshold_distance)
    euclidean_distance = sqrt((p1_x - p2_x)^2 + (p1_y - p2_y)^2)
    if(euclidean_distance<=threshold_distance)
        return true
    else
        return false
    end
end

function in_obstacle(px,py,obstacle,padding=0.0)
    return is_within_range(px,py,obstacle.x,obstacle.y,obstacle.r+padding)
end


#Heuristic Cost
struct heuristic_cost <: Function
    env::ExperimentEnvironment
end

function (obj::heuristic_cost)(goal, vehicle_state)
    world = obj.env
    euclidean_distance =  sqrt( (vehicle_state.x - goal.x)^2 + (vehicle_state.y - goal.y)^2 )
    direct_line_to_goal_slope = wrap_between_0_and_2Pi(atan(goal.y-vehicle_state.y,goal.x-vehicle_state.x))
    orientation_cost = 10* dot( (cos(direct_line_to_goal_slope), sin(direct_line_to_goal_slope)) ,
                                (cos(vehicle_state.theta), sin(vehicle_state.theta)) )
    return euclidean_distance - orientation_cost
end


function main()
    e = ExperimentEnvironment(100.0,100.0,ObstacleLocation[ObstacleLocation(30.0,30.0,15.0),ObstacleLocation(70.0,60.0,15.0)])
    holonomic_vs = VehicleState(10.0,20.0,0.0)
    holonomic_va = get_vehicle_actions(45,5)
    g = Location(75.0,95.0)
    nc = node_cost(0.5, e)
    hc = heuristic_cost(e)
    cs = hybrid_astar_search(g, holonomic_vs, holonomic_va, holonomic_vehicle_dynamics, get_node_key, nc, hc)
    return cs
end


#=
mutable struct PathPlanningDetails
    num_nearby_humans::Int64
    radius_around_uncertain_human::Float64
    min_safe_distance_from_human::Float64
    human_collision_cost::Float64
    human_goals::Array{Location,1}
    veh_path_planning_v::Float64
    radius_around_vehicle_goal::Float64
    lambda::Float64
    one_time_step::Float64
    planning_time::Float64
end


w = generate_environment(100.0,100.0,ObstacleLocation[ObstacleLocation(30.0,30.0,15.0),ObstacleLocation(70.0,60.0,15.0)])
pd = PathPlanningDetails(0,20.0,1.0,100.0,Location[],1.0,1.0,0.99,0.5,10.0)
v = Vehicle(5.0,10.0,0.0,pd.veh_path_planning_v)
vp = VehicleParametersLSPlanner(0.3,3.0,Location(98.0,80.0),Float64[])
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
va = get_vehicle_actions(45,5)
cs = hybrid_astar_search(w, v, vp, va, nbh, pd);
new_vp = VehicleParametersLSPlanner(vp.L,vp.max_speed,vp.goal,cs)


w = generate_environment(6.0,12.0,ObstacleLocation[ObstacleLocation(3.0,6.0,1.0)])
pd = PathPlanningDetails(0,20.0,1.0,100.0,Location[],1.0,1.0,0.99,0.5,1.0)
v = Vehicle(0.5,0.5,0.0,pd.veh_path_planning_v)
vp = VehicleParametersLSPlanner(0.3,3.0,Location(3.0,11.5),Float64[])
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
va = get_vehicle_actions(45,15)
cs = hybrid_astar_search(w, v, vp, va, nbh, pd);
new_vp = VehicleParametersLSPlanner(vp.L,vp.max_speed,vp.goal,cs)

va = get_vehicle_actions(170,10)
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
p = hybrid_astar_search(env, veh, veh_params, va, nbh, path_planning_details)
new_params = modify_vehicle_params(veh_params, p)
px,py,pt = get_hybrid_astar_trajectory(veh,new_params,1,path_planning_details,exp_details)
=#

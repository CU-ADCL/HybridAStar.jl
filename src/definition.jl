struct GraphNode{ST,AT,KT}
    state::ST
    key::KT
    actual_cost::Float64
    heuristic_cost::Float64
    parent_key::Union{KT,Nothing}
    parent_action::Union{AT,Nothing}
    depth::Int64
end

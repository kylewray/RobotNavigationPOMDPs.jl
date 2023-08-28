@with_kw mutable struct RobotNavigationPose
    x::Real = 0.0
    y::Real = 0.0
    θ::Real = 0.0
end

@with_kw mutable struct RobotNavigationState
    pose::RobotNavigationPose = RobotNavigationPose()
    map_name::String = "map"
    task_name::String = "task"
end

@with_kw mutable struct RobotNavigationAction
    desired_move::Bool = false
    desired_θ::Real = 0.0
end

@with_kw mutable struct RobotNavigationObservation
    depths::Vector{<:Real} = []
    objects::Vector{<:Int} = []
end

const RobotNavigationStates = Vector{RobotNavigationState}
const RobotNavigationActions = Vector{RobotNavigationAction}
const RobotNavigationObservations = Vector{RobotNavigationObservation}


@with_kw mutable struct RobotNavigationPOMDP <: POMDP{RobotNavigationState,
                                                      RobotNavigationAction,
                                                      RobotNavigationObservation}
    map_names::Vector{String} = ["map"]
    size_of_map::Dict{String, NamedTuple{(:width, :height), Tuple{Int, Int}}} = Dict("map" => (width = 10, height = 10))
    meters_per_pixel::Real = 1.0
    num_determinized_orientations::Int = 4
    room_names_for_map::Dict{String, Vector{String}} = Dict("map" => ["room"])
    task_names_for_map::Dict{String, Vector{String}} = Dict("map" => ["task"])
    max_xy_speed::Real = 1.0
    max_θ_speed::Real = float(π) / 4.0
    num_depths::Int = 4
    max_depth_range::Real = 1.0
    num_objects::Int = 0
end


function POMDPs.states(𝒫::RobotNavigationPOMDP)
    𝒮::RobotNavigationStates = []
    for m in 𝒫.map_names
        for x in 1:floor(Int, 𝒫.size_of_map[m].width * 𝒫.meters_per_pixel)
            for y in 1:floor(Int, 𝒫.size_of_map[m].height * 𝒫.meters_per_pixel)
                for o in 1:𝒫.num_determinized_orientations
                    for t in 𝒫.task_names_for_map[m]
                        s = RobotNavigationState(
                            pose = RobotNavigationPose(
                                x = x,
                                y = y,
                                θ = loop_angle(2.0 * π * (o - 1) / 𝒫.num_determinized_orientations)
                            ),
                            map_name = m,
                            task_name = t,
                        )
                        push!(𝒮, s)
                    end
                end
            end
        end
    end
    return 𝒮
end


function POMDPs.stateindex(𝒫::RobotNavigationPOMDP, s::RobotNavigationState)
    # TODO: Make this more efficient later.

    𝒮 = states(𝒫)
    for (i, s′) in 𝒮
        if s′ == s
            return i
        end
    end

    return missing
end


function POMDPs.actions(𝒫::RobotNavigationPOMDP)
    𝒜::RobotNavigationActions = []
    for m in [false, true]
        for o in 1:𝒫.num_determinized_orientations
            a = RobotNavigationAction(
                desired_move = m,
                desired_θ = loop_angle(2.0 * π * (o - 1) / 𝒫.num_determinized_orientations),
            )
            push!(𝒜, a)
        end
    end
    return 𝒜
end


function POMDPs.actionindex(𝒫::RobotNavigationPOMDP, a::RobotNavigationAction)
    # TODO: Make this more efficient later.

    𝒜 = actions(𝒫)
    for (i, a′) in 𝒜
        if a′ == a
            return i
        end
    end

    return missing
end


function POMDPs.observations(𝒫::RobotNavigationPOMDP)
    𝒪::RobotNavigationObservations = []
    depth_factors = [[d for d in 0:floor(Int, 𝒫.max_depth_range)]
                     for i in 1:𝒫.num_depths]
    for d in permutations_with_replacement(depth_factors)
        for j in powerset(1:𝒫.num_objects)
            o = RobotNavigationObservation(
                depths = collect(d),
                objects = j,
            )
            push!(𝒪, o)
        end
    end
    return 𝒪
end


function POMDPs.transition(𝒫::RobotNavigationPOMDP, s::RobotNavigationState, a::RobotNavigationAction)
    s′ = deepcopy(s)
    a′ = deepcopy(a)

    changed_θ = abs(s′.pose.θ - a′.desired_θ) > 0.01

    if !a′.desired_move && !changed_θ
        return Deterministic(s′)
    end

    𝒩xy = Normal(𝒫.max_xy_speed, 0.1)
    𝒩θ = Normal(
        clamp(
            turn_to_angle(s′.pose.θ, a′.desired_θ),
            -𝒫.max_θ_speed,
            𝒫.max_θ_speed
        ),
        0.1
    )

    𝒯₁(rng) = begin
        s′.pose.x += rand(rng, 𝒩xy) * cos(s′.pose.θ)
        s′.pose.y += rand(rng, 𝒩xy) * sin(s′.pose.θ)
        return s′
    end
    𝒯₂(rng) = begin
        s′.pose.θ += rand(rng, 𝒩θ)
        return s′
    end
    𝒯₃(rng) = begin
        # Change heading first, then move.
        s′.pose.θ = rand(rng, 𝒩θ)
        s′.pose.x += rand(rng, 𝒩xy) * cos(s′.pose.θ)
        s′.pose.y += rand(rng, 𝒩xy) * sin(s′.pose.θ)
        return s′
    end

    if a.desired_move && !changed_θ
        return ImplicitDistribution(𝒯₁)
    end

    if !a.desired_move && changed_θ
        return ImplicitDistribution(𝒯₂)
    end

    return ImplicitDistribution(𝒯₃)
end


@with_kw mutable struct RobotNavigationMap
    map_name::Symbol = :map
    absolute_path::String = ""
    image # TODO: Figure out what this type is!
    image_width::Int = 0
    image_height::Int = 0

    function RobotNavigationMap(map_name::Symbol, filename::String)
        absolute_path = joinpath(@__DIR__, "..", "maps", filename)
        image = load(absolute_path)
        w, h = size(image, 2), size(image, 1)
        return new(map_name, absolute_path, image, w, h)
    end
end

@enum RobotNavigationColor begin
    BLACK
    WHITE
    RED
    GREEN
    BLUE
    MAGENTA
    YELLOW
    CYAN
end

const global OBSERVABLE_COLORS = (BLACK, WHITE, RED, GREEN, BLUE)
const global TASK_COLORS = (MAGENTA, YELLOW)
const global INITIAL_STATE_COLOR = CYAN
const global ALL_COLORS = (BLACK, WHITE, RED, GREEN, BLUE, MAGENTA, YELLOW, CYAN)

@with_kw mutable struct RobotNavigationPose
    x::Real = 0.0
    y::Real = 0.0
    θ::Real = 0.0
end

@with_kw mutable struct RobotNavigationState
    pose::RobotNavigationPose = RobotNavigationPose()
    map_name::Symbol = :map
    task_color::RobotNavigationColor = MAGENTA
end

@with_kw mutable struct RobotNavigationAction
    desired_move::Bool = false
    desired_θ::Real = 0.0
end

@with_kw mutable struct RobotNavigationScan
    ϕ::Real = 0.0       # Relative angle to robot's pose.
    depth::Real = 0.0   # Relative depth to robot's pose.
    color::RobotNavigationColor = WHITE
end

@with_kw mutable struct RobotNavigationObservation
    scans::Vector{RobotNavigationScan} = []
end

const RobotNavigationStates = Vector{RobotNavigationState}
const RobotNavigationActions = Vector{RobotNavigationAction}
const RobotNavigationObservations = Vector{RobotNavigationObservation}

@with_kw mutable struct RobotNavigationPOMDP <: POMDP{RobotNavigationState,
                                                      RobotNavigationAction,
                                                      RobotNavigationObservation}
    maps::Dict{Symbol, RobotNavigationMap} = Dict(
        :map => RobotNavigationMap(:map, "simple_test.png")
    )
    size_of_map::Dict{Symbol, NamedTuple{(:width, :height), Tuple{Int, Int}}} = Dict(
        :map => (width = 10, height = 10)
    )
    meters_per_pixel::Real = 1.0
    num_determinized_orientations::Int = 4 # NOTE: For deterministic only.
    max_tasks_per_map::Int = 1
    task_color_for_map::Dict{Symbol, Vector{RobotNavigationColor}} = Dict(
        :map => [MAGENTA]
    )
    robot_radius::Real = 0.25
    move_xy_max_speed::Real = 1.0
    move_θ_max_speed::Real = float(π) / 4.0
    move_xy_variance::Real = 0.1
    move_θ_variance::Real = 0.1
    num_scans::Int = 3
    scan_field_of_view::Real = float(π) / 2.0
    scan_range::Real = 5.0
    scan_depth_variance::Real = 0.25
    scan_color_observation_probability::Real = 0.9
end


@with_kw mutable struct RobotNavigationObservationDistribution
    𝒫::RobotNavigationPOMDP
    a::RobotNavigationAction
    s′::RobotNavigationState
end


function color_to_rgb(color::RobotNavigationColor)
    if color == BLACK
        return (r = 0.0, g = 0.0, b = 0.0)
    elseif color == WHITE
        return (r = 1.0, g = 1.0, b = 1.0)
    elseif color == GREEN
        return (r = 0.0, g = 1.0, b = 0.0)
    elseif color == RED
        return (r = 1.0, g = 0.0, b = 0.0)
    elseif color == BLUE
        return (r = 0.0, g = 0.0, b = 1.0)
    elseif color == MAGENTA
        return (r = 1.0, g = 0.0, b = 1.0)
    elseif color == YELLOW
        return (r = 1.0, g = 1.0, b = 0.0)
    elseif color == CYAN
        return (r = 0.0, g = 1.0, b = 1.0)
    else
        return (r = 0.3333, g = 0.333, b = 0.33)
    end
end


function iscolor(𝒫::RobotNavigationPOMDP, s::RobotNavigationState, color::RobotNavigationColor)
    image = 𝒫.maps[s.map_name].image
    height = 𝒫.maps[s.map_name].image_height
    width = 𝒫.maps[s.map_name].image_width

    y = floor(Int, s.pose.y / 𝒫.meters_per_pixel) + 1
    x = floor(Int, s.pose.x / 𝒫.meters_per_pixel) + 1

    if y < 1 || y > height || x < 1 || x > width
        if color == BLACK
            return true
        else
            return false
        end
    end

    c = color_to_rgb(color)

    result = (
        c.r == convert(Float64, image[y, x].r)
        && c.g == convert(Float64, image[y, x].g)
        && c.b == convert(Float64, image[y, x].b)
    )

    return result
end


function POMDPs.states(𝒫::RobotNavigationPOMDP)
    𝒮::RobotNavigationStates = []
    for (m, map) in 𝒫.maps
        for x in 1:floor(Int, 𝒫.size_of_map[m].width * 𝒫.meters_per_pixel)
            for y in 1:floor(Int, 𝒫.size_of_map[m].height * 𝒫.meters_per_pixel)
                for o in 1:𝒫.num_determinized_orientations
                    for t in 𝒫.task_color_for_map[m]
                        s = RobotNavigationState(
                            pose = RobotNavigationPose(
                                x = x,
                                y = y,
                                θ = loop_angle(2.0 * π * (o - 1) / 𝒫.num_determinized_orientations)
                            ),
                            map_name = m,
                            task_color = t,
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
    depth_factors = [[
            0, 𝒫.scan_range
        ] for i in 1:𝒫.num_scans
    ]

    color_factors = [[
            c for c in OBSERVABLE_COLORS
        ] for i in 1:𝒫.num_scans
    ]
    ϕhalf = 𝒫.scan_field_of_view / 2.0
    for d in permutations_with_replacement(depth_factors)
        for c in permutations_with_replacement(color_factors)
            depth_colors = []

            ϕstep = 𝒫.scan_field_of_view / (𝒫.num_scans - 1)
            for (i, ϕ) in enumerate(-ϕhalf:ϕstep:ϕhalf)
                push!(
                    depth_colors,
                    RobotNavigationScan(
                        ϕ = loop_angle(ϕ),
                        depth = d[i],
                        color = c[i]
                    )
                )
            end
            o = RobotNavigationObservation(depth_colors)
            push!(𝒪, o)
        end
    end
    return 𝒪
end


function POMDPs.transition(𝒫::RobotNavigationPOMDP, s::RobotNavigationState, a::RobotNavigationAction)
    s′′ = deepcopy(s)
    changed_θ = difference_of_angles(s.pose.θ, a.desired_θ) > 0.01

    if !a.desired_move && !changed_θ
        return Deterministic(s′′)
    end

    # The xy noise is in meters and the θ noise is in radians.
    𝒩xy = Normal(0.0, 𝒫.move_xy_variance)
    𝒩θ = Normal(
        clamp(
            turn_to_angle(s.pose.θ, a.desired_θ),
            -𝒫.move_θ_max_speed,
            𝒫.move_θ_max_speed
        ),
        𝒫.move_θ_variance
    )

    move(rng, s′) = begin
        # Distance is in meters, but the for loop is over mainly pixels.
        # Pixels should be bigger jumps, and since we are only detecting
        # obstacles over them, we set the max iterations to be based
        # on a sub-pixel width, and allow for the step to be random.
        max_distance = 𝒫.move_xy_max_speed
        sub_pixel_multiplier = 0.1
        max_iterations = max(
            𝒫.maps[s′.map_name].image_width,
            𝒫.maps[s′.map_name].image_height
        ) / sub_pixel_multiplier

        # NOTE: Always start the robot at its radius to prevent collisions.
        distance = 𝒫.robot_radius
        for i in 1:max_iterations
            # We step forward in sub-pixel meters.
            step_size = sub_pixel_multiplier * 𝒫.meters_per_pixel

            s′.pose.x += step_size * cos(s′.pose.θ)
            s′.pose.y += step_size * sin(s′.pose.θ)
            distance += step_size

            # If this is any observable color that is not white,
            # then we can collide with it too.
            if (any(iscolor(𝒫, s′, c)
                    for c in OBSERVABLE_COLORS
                    if c != WHITE)
                || distance >= max_distance
            )
                s′.pose.x -= step_size * cos(s′.pose.θ)
                s′.pose.y -= step_size * sin(s′.pose.θ)
                distance -= step_size
                break
            end
        end

        # Attempt to add random noise to this step length. However,
        # if it encounters a wall, undo this random step.
        random_step_size = rand(rng, 𝒩xy)
        s′.pose.x += random_step_size * cos(s′.pose.θ)
        s′.pose.y += random_step_size * sin(s′.pose.θ)
        if iscolor(𝒫, s′, BLACK) 
            s′.pose.x -= random_step_size * cos(s′.pose.θ)
            s′.pose.y -= random_step_size * sin(s′.pose.θ)
        end

        return s′
    end
    turn(rng, s′) = begin
        s′.pose.θ = loop_angle(s′.pose.θ + rand(rng, 𝒩θ))
        return s′
    end
    turn_then_move(rng, s′) = begin
        s′.pose.θ = loop_angle(s′.pose.θ + rand(rng, 𝒩θ))
        return move(rng, s′)
    end

    deterministically_progress_task(s′) = begin
        for (i, c) in enumerate(𝒫.task_color_for_map[s′.map_name])
            if iscolor(𝒫, s′, c) && s′.task_color == c
                if i < length(𝒫.task_color_for_map[s′.map_name])
                    s′.task_color = 𝒫.task_color_for_map[s′.map_name][i + 1]
                else
                    s′.task_color = 𝒫.task_color_for_map[s′.map_name][1]
                end
                break
            end
        end
        return s′
    end
    deterministically_progress_task(s′′)

    if a.desired_move && !changed_θ
        return ImplicitDistribution(rng -> move(rng, s′′))
    end

    if !a.desired_move && changed_θ
        return ImplicitDistribution(rng -> turn(rng, s′′))
    end

    return ImplicitDistribution(rng -> turn_then_move(rng, s′′))
end


function deterministic_observation(𝒫::RobotNavigationPOMDP, a::RobotNavigationAction, s′::RobotNavigationState)
    s′′ = deepcopy(s′)

    deterministic_color(s′′′) = begin
        # These colors are the observable ones. Other colors
        # are not observable and are used for tasks.
        for c in OBSERVABLE_COLORS
            if iscolor(𝒫, s′′′, c)
                return c
            end
        end

        # If this s′′′ was in a non-observable color, then it
        # is treated as empty space (white).
        return WHITE
    end

    deterministic_depth_color(ϕ, s′′′) = begin
        # Depth is in meters, but the for loop is over mainly pixels.
        # Pixels should be bigger jumps, and since we are only detecting
        # obstacles over them, we set the max iterations to be based
        # on a sub-pixel width, and allow for the step to be random.
        # NOTE: We use scan range instead of max depth (transition's move).
        max_depth = 𝒫.scan_range
        sub_pixel_multiplier = 0.1
        max_iterations = max(
            𝒫.maps[s′′′.map_name].image_width,
            𝒫.maps[s′′′.map_name].image_height
        ) / sub_pixel_multiplier

        depth = 0.0
        for i in 1:max_iterations
            # We step forward in sub-pixel meters.
            step_size = sub_pixel_multiplier * 𝒫.meters_per_pixel

            s′′′.pose.x += step_size * cos(s′′′.pose.θ + ϕ)
            s′′′.pose.y += step_size * sin(s′′′.pose.θ + ϕ)
            depth += step_size

            # If this is any observable color that is not white,
            # then we can collide with it too.
            if (any(iscolor(𝒫, s′′′, c)
                    for c in OBSERVABLE_COLORS
                    if c != WHITE)
                || depth >= max_depth
            )
                break
            end
        end

        # NOTE: This s′′′ is updated to be in collision or max depth.
        return RobotNavigationScan(ϕ, depth, deterministic_color(s′′′))
    end

    scans = []

    ϕhalf = 𝒫.scan_field_of_view / 2.0
    if 𝒫.num_scans > 1 
        ϕstep = 𝒫.scan_field_of_view / (𝒫.num_scans - 1)
        for ϕ in -ϕhalf:ϕstep:ϕhalf
            s′′′ = deepcopy(s′′)
            push!(scans, deterministic_depth_color(loop_angle(ϕ), s′′′))
        end
    else
        s′′′ = deepcopy(s′′)
        push!(scans, deterministic_depth_color(0.0, s′′′))
    end

    return RobotNavigationObservation(scans)
end


function deterministic_observation(od::RobotNavigationObservationDistribution)
    return deterministic_observation(od.𝒫, od.a, od.s′)
end


function rand(rng::AbstractRNG, od::RobotNavigationObservationDistribution)
    o = deterministic_observation(od)

    for i in 1:length(o.scans)
        # The ϕ angle is already set and is fixed.
        #o.scans[i].ϕ = FIXED

        if rand(rng) >= od.𝒫.scan_color_observation_probability
            o.scans[i].color = rand(rng, OBSERVABLE_COLORS)
        end

        𝒩d = Normal(o.scans[i].depth, od.𝒫.scan_depth_variance)
        o.scans[i].depth = rand(rng, 𝒩d)
    end

    return o
end


function POMDPs.pdf(od::RobotNavigationObservationDistribution, o::RobotNavigationObservation)
    o′ = deterministic_observation(od)

    p = 1.0

    for i in 1:length(o.scans)
        if o.scans[i].color == o′.scans[i].color
            p *= (
                od.𝒫.scan_color_observation_probability
                + (1.0 - od.𝒫.scan_color_observation_probability) * 1.0 / length(OBSERVABLE_COLORS)
            )
        else
            p *= 1.0 / length(OBSERVABLE_COLORS)
        end

        𝒩d = Normal(o′.scans[i].depth, od.𝒫.scan_depth_variance)
        p *= POMDPs.pdf(𝒩d, o.scans[i].depth)
    end

    return p
end


function POMDPs.observation(𝒫::RobotNavigationPOMDP, a::RobotNavigationAction, s′::RobotNavigationState)
    return RobotNavigationObservationDistribution(𝒫, a, s′)
end


function POMDPs.reward(𝒫::RobotNavigationPOMDP, s::RobotNavigationState, a::RobotNavigationAction)
    if iscolor(𝒫, s, s.task_color)
        return 0.0
    else
        return -1.0
    end
end


function POMDPs.initialstate(𝒫::RobotNavigationPOMDP)
    random_initial_pixel(rng, θ, map_name, task_color) = begin
        image = 𝒫.maps[map_name].image
        height = round(Int, size(image, 1))
        width = round(Int, size(image, 2))

        initial_pixels = []
        for py in 1:height
            for px in 1:width
                s = RobotNavigationState(
                    RobotNavigationPose(
                        px * 𝒫.meters_per_pixel,
                        py * 𝒫.meters_per_pixel,
                        θ
                    ),
                    map_name,
                    task_color
                )
                if iscolor(𝒫, s, INITIAL_STATE_COLOR)
                    push!(initial_pixels, (y = py, x = px))
                end
            end
        end

        return rand(rng, initial_pixels)
    end

    b(rng) = begin
        map_name, map = rand(rng, 𝒫.maps)
        task_color = rand(rng, 𝒫.task_color_for_map[map_name])
        θ = rand(rng) * 2.0 * π

        pixel = random_initial_pixel(rng, θ, map_name, task_color)

        x = (pixel.x + rand(rng) * 0.5 + 0.25) * 𝒫.meters_per_pixel
        y = (pixel.y + rand(rng) * 0.5 + 0.25) * 𝒫.meters_per_pixel

        return RobotNavigationState(
            RobotNavigationPose(x, y, θ),
            map_name,
            task_color
        )
    end

    return ImplicitDistribution(b)
end


POMDPs.discount(𝒫::RobotNavigationPOMDP) = 0.95
POMDPs.isterminal(𝒫::RobotNavigationPOMDP) = false


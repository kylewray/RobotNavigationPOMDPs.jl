function prettystring(s::RobotNavigationState)
    return "RobotNavigationState(pose = ($(s.pose.x), $(s.pose.y), $(s.pose.θ)), map_name = $(s.map_name), task_color = $(s.task_color))"
end


#Base.show(io::IO, s::RobotNavigationState) = println(io, prettystring(s))
Base.show(io::IO, ::MIME"text/plain", s::RobotNavigationState) = println(io, prettystring(s))


function prettystring(a::RobotNavigationAction)
    return "RobotNavigationAction(desired_move = $(a.desired_move), desired_θ = $(a.desired_θ))"
end


#Base.show(io::IO, a::RobotNavigationAction) = println(io, prettystring(a))
Base.show(io::IO, ::MIME"text/plain", a::RobotNavigationAction) = println(io, prettystring(a))


function prettystring(o::RobotNavigationObservation)
    result = "RobotNavigationObservation(depths = [\n"
    for i in 1:length(o.scans)
        result *= "    depth = $(o.scans[i].depth), color = $(o.scans[i].color)\n"
    end
    result *= "]"
    return result
end


#Base.show(io::IO, o::RobotNavigationObservation) = println(io, prettystring(o))
Base.show(io::IO, ::MIME"text/plain", o::RobotNavigationObservation) = println(io, prettystring(o))


function transform_coordinates(𝒫::RobotNavigationPOMDP, s::RobotNavigationState)
    x, y, θ = s.pose.x, s.pose.y, s.pose.θ
    #map_name = s.map_name

    #image = 𝒫.maps[map_name].image
    #height = 𝒫.maps[map_name].image_height
    #width = 𝒫.maps[map_name].image_width

    yImage = y / 𝒫.meters_per_pixel
    xImage = x / 𝒫.meters_per_pixel
    θImage = θ

    return xImage, yImage, θImage
end


struct RobotNavigationVisualizer
    𝒫::RobotNavigationPOMDP
    step::Any
    text::String
end


robot_navigation_visualizer(𝒫::RobotNavigationPOMDP, step::Any; text::String = "") = RobotNavigationVisualizer(𝒫, step, text)


function render_robot_action(m::Any, 𝒫::RobotNavigationPOMDP, step::Any)
    if !haskey(step, :a)
        return
    end

    a = step[:a]

    # TODO
end


function render_robot_state_prime(m::Any, 𝒫::RobotNavigationPOMDP, step::Any)
    if !haskey(step, :sp)
        return
    end

    sp = step[:sp]

    # Compute the robot and particle size, in *Figure*-pixel space (not image-pixel space).
    figure_pixels_per_image_pixels = (
        min(m.w, m.h)
        / min(𝒫.maps[sp.map_name].image_width, 𝒫.maps[sp.map_name].image_width)
    )
    marker_size = (
        (2.0 * 𝒫.robot_radius)
        / 𝒫.meters_per_pixel
        * figure_pixels_per_image_pixels
    )

    iterator = 1
    for (map_name, map) in 𝒫.maps
        ax = m.axes[iterator]

        # Render the map image.
        # NOTE: The `'` tick mark is key here. The `image` is a `Matrix`.
        # The `'` tick mark transposes the image properly to render it.
        # NOTE: The `interpolate` setting is to set it to `NEAREST` and
        # become a crisp pixel image.
        # TODO: Use step[:bp] to get an alpha-ed distribution over the maps.
        image!(ax, 𝒫.maps[map_name].image', interpolate = false)

        # Render the particle beliefs (if any).
        if haskey(step, :bp)
            bp = step[:bp]
            if bp isa AbstractParticleBelief
                for spb in particles(bp)
                    if map_name == spb.map_name
                        x, y, θ = transform_coordinates(𝒫, spb)

                        scatter!(
                            ax,
                            [x],
                            [y],
                            rotations = [θ],
                            color = :blue,
                            alpha = 0.2,
                            marker = :rtriangle, #'→',
                            markersize = marker_size,
                            #markerspace = :data,
                        )
                    end
                end
            end
        end

        # Render the robot base circle, if this is the true map.
        if map_name == sp.map_name
            x, y, θ = transform_coordinates(𝒫, sp)

            scatter!(
                ax,
                [x],
                [y],
                rotations = [θ],
                color = :green,
                #alpha = 1.0,
                marker = :rtriangle, #'→',
                markersize = marker_size,
                #markerspace = :data,
            )
        end

        iterator += 1
    end
end


function render_robot_observation(m::Any, 𝒫::RobotNavigationPOMDP, step::Any)
    if !haskey(step, :o) || !haskey(step, :sp)
        return
    end

    sp = step[:sp]
    o = step[:o]

    # Compute the robot and particle size, in *Figure*-pixel space (not image-pixel space).
    figure_pixels_per_image_pixels = (
        min(m.w, m.h)
        / min(𝒫.maps[sp.map_name].image_width, 𝒫.maps[sp.map_name].image_width)
    )
    marker_size = (
        (2.0 * 𝒫.robot_radius)
        / 𝒫.meters_per_pixel
        * figure_pixels_per_image_pixels
    )

    # Figure out which map the robot is really in.
    iterator = 1
    for (map_name, map) in 𝒫.maps
        if map_name == sp.map_name
            break
        end
        iterator += 1
    end

    # Render all of the observations, starting from the robot's true state.
    x, y, θ = transform_coordinates(𝒫, sp)

    for scan in o.scans
        lines!(
            m.axes[iterator],
            [x, x + scan.depth * cos(θ + scan.ϕ)],
            [y, y + scan.depth * sin(θ + scan.ϕ)],
            color = :black,
            alpha = 0.5,
            linewidth = marker_size / 10.0,
            linestyle = :dot
        )
    end

    for scan in o.scans
        sc = :white

        if scan.color == BLACK
            c = :black
        elseif scan.color == WHITE
            c = :white
            sc = :black
        elseif scan.color == GREEN
            c = :green
        elseif scan.color == RED
            c = :red
        elseif scan.color == BLUE
            c = :blue
        elseif scan.color == YELLOW
            c = :yellow
        elseif scan.color == CYAN
            c = :cyan
        elseif scan.color == MAGENTA
            c = :magenta
        else
            c = :gray
        end

        scatter!(
            m.axes[iterator],
            [x + scan.depth * cos(θ + scan.ϕ)],
            [y + scan.depth * sin(θ + scan.ϕ)],
            rotations = [θ + scan.ϕ],
            color = c,
            #alpha = 1.0,
            marker = :circle, #'→',
            markersize = marker_size / 2.0,
            strokewidth = marker_size / 23.0,
            strokecolor = sc,
        )
    end
end


function robot_navigation_show(𝒱::RobotNavigationVisualizer)
    base_figure_size_in_pixels = 1000

    aspect_ratios = []
    for (name, map) in 𝒱.𝒫.maps
        push!(aspect_ratios, map.image_width / map.image_height)
    end
    max_aspect_ratio = maximum(aspect_ratios)

    w = base_figure_size_in_pixels
    h = base_figure_size_in_pixels * max_aspect_ratio

    fig = Figure(size = (w, h))

    axes = []
    iterator = 1
    for (map_name, map) in 𝒱.𝒫.maps
        fig_r = floor(Int, iterator / 2) + 1
        fig_c = (iterator - 1) % 2 + 1

        # NOTE: Both `Images` and `Makie` have `Axis`.
        ax = Makie.Axis(
            fig[fig_r, fig_c],
            #yreversed = true,
            title = string(map_name)
        )
        push!(axes, ax)

        iterator += 1
    end

    m = (fig = fig, axes = axes, w = w, h = h)
    render_robot_action(m, 𝒱.𝒫, 𝒱.step)
    render_robot_state_prime(m, 𝒱.𝒫, 𝒱.step)
    render_robot_observation(m, 𝒱.𝒫, 𝒱.step)

    return fig
end


Base.show(io::IO, mime::Union{MIME"text/html", MIME"image/svg+xml"}, 𝒱::RobotNavigationVisualizer) = robot_navigation_show(𝒱)


# TODO: Figure out saving using `FileIO`'s (?) `save` instead of `Makie.save`
# For now, you can still do: `Makie.save("my_file.pdf", robot_navigation_show(𝒱))`.
#function Base.show(io::IO, mime::MIME"image/png", 𝒱::RobotNavigationVisualizer)
#    fig = robot_navigation_show(𝒱)
#    return save(io, fig)
#end


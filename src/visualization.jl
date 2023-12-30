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


struct RobotNavigationVisualizer
    𝒫::RobotNavigationPOMDP
    step::Any
    show_most_likely_map::Bool
    show_current_state::Bool
    text::String
end


robot_navigation_visualizer(
    𝒫::RobotNavigationPOMDP,
    step::Any;
    show_most_likely_map = false,
    show_current_state = false,
    text::String = ""
) = RobotNavigationVisualizer(𝒫, step, show_most_likely_map, show_current_state, text)


sorted_map_names(𝒫::RobotNavigationPOMDP) = sort([mn for mn in keys(𝒫.maps)])


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
    for map_name in sorted_map_names(𝒫)
        map = 𝒫.maps[map_name]
        ax = m.axes[iterator]

        # Render the map image.
        # NOTE: The `'` tick mark is key here. The `image` is a `Matrix`.
        # The `'` tick mark transposes the image properly to render it.
        # NOTE: The `interpolate` setting is to set it to `NEAREST` and
        # become a crisp pixel image.
        # TODO: Use step[:bp] to get an alpha-ed distribution over the maps.
        image!(
            ax, 
            (0, 𝒫.maps[map_name].image_width * 𝒫.meters_per_pixel),
            (0, 𝒫.maps[map_name].image_height * 𝒫.meters_per_pixel),
            𝒫.maps[map_name].image',
            interpolate = false
        )

        # Render the particle beliefs (if any).
        if haskey(step, :bp)
            bp = step[:bp]
            if bp isa AbstractParticleBelief
                for spb in particles(bp)
                    if map_name == spb.map_name
                        x, y, θ = spb.pose.x, spb.pose.y, spb.pose.θ

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
            x, y, θ = sp.pose.x, sp.pose.y, sp.pose.θ

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
    for map_name in sorted_map_names(𝒫)
        map = 𝒫.maps[map_name]
        if map_name == sp.map_name
            break
        end
        iterator += 1
    end

    # Render all of the observations, starting from the robot's true state.
    x, y, θ = sp.pose.x, sp.pose.y, sp.pose.θ

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

    map_names = sorted_map_names(𝒱.𝒫)

    aspect_ratios = []
    for map_name in map_names
        map = 𝒱.𝒫.maps[map_name]
        push!(aspect_ratios, map.image_width / map.image_height)
    end
    max_aspect_ratio = maximum(aspect_ratios)

    if !𝒱.show_most_likely_map
        w = base_figure_size_in_pixels
        h = base_figure_size_in_pixels * max_aspect_ratio
    else
        w = base_figure_size_in_pixels * 2.0
        h = base_figure_size_in_pixels * max_aspect_ratio
    end

    fig = Figure(size = (w, h))

    axes = []

    if !𝒱.show_most_likely_map
        iterator, fig_r, fig_c = 1, 1, 1
        for map_name in map_names
            map = 𝒱.𝒫.maps[map_name]

            # NOTE: Both `Images` and `Makie` have `Axis`.
            ax = Makie.Axis(
                fig[fig_r, fig_c],
                #yreversed = true,
                title = string(map_name),
                xlabel = "x (meters)",
                ylabel = "y (meters)",
            )
            push!(axes, ax)

            iterator += 1
            fig_c += 1
            if fig_c > 2
                fig_c = 1
            end
            if iterator % 2 == 1
                fig_r += 1
            end
        end
    else
        # Determine the most likely map.
        counts = Dict(mn => 0.0 for mn in map_names)

        if !𝒱.show_current_state && haskey(𝒱.step, :bp)
            bp = 𝒱.step[:bp]
            if bp isa AbstractParticleBelief
                for spb in particles(bp)
                    counts[spb.map_name] += 1
                end
            end
        elseif haskey(𝒱.step, :b)
            b = 𝒱.step[:b]
            if b isa AbstractParticleBelief
                for sb in particles(b)
                    counts[sb.map_name] += 1
                end
            end
        end

        most_likely_map_name = last(findmax(counts))

        # How many figure blocks to reserve (a square).
        most_likely_blocks = ceil(Int, length(map_names) / 2)

        # Setup the figures to highlight the most likely map.
        iterator, fig_r, fig_c = 1, 1, 1
        for map_name in map_names
            map = 𝒱.𝒫.maps[map_name]

            if map_name != most_likely_map_name
                # NOTE: Both `Images` and `Makie` have `Axis`.
                ax = Makie.Axis(
                    fig[fig_r, most_likely_blocks + fig_c],
                    #yreversed = true,
                    title = string(map_name),
                    xlabel = "x (meters)",
                    ylabel = "y (meters)",
                )
                push!(axes, ax)

                iterator += 1

                fig_c += 1
                if fig_c > 2
                    fig_c = 1
                end
                if iterator % 2 == 1
                    fig_r += 1
                end
            else
                # NOTE: Both `Images` and `Makie` have `Axis`.
                ax = Makie.Axis(
                    fig[1:most_likely_blocks, 1:most_likely_blocks],
                    #yreversed = true,
                    title = string(map_name),
                    xlabel = "x (meters)",
                    ylabel = "y (meters)",
                )
                push!(axes, ax)
            end
        end
    end

    m = (fig = fig, axes = axes, w = w, h = h)

    # Normally, render the resulting successor state, belief, observation,
    # and so on. Otherwise, show the current state, action, and so on.
    if !𝒱.show_current_state
        render_robot_action(m, 𝒱.𝒫, 𝒱.step)
        render_robot_state_prime(m, 𝒱.𝒫, 𝒱.step)
        render_robot_observation(m, 𝒱.𝒫, 𝒱.step)
    elseif haskey(𝒱.step, :s) && haskey(𝒱.step, :b)
        step′ = Dict(
            :sp => deepcopy(𝒱.step[:s]),
            :bp => deepcopy(𝒱.step[:b]),
            :o => deepcopy(𝒱.step[:o]),
        )
        render_robot_state_prime(m, 𝒱.𝒫, step′)
        render_robot_action(m, 𝒱.𝒫, step′)
    end

    return fig
end


Base.show(
    io::IO,
    mime::Union{MIME"text/html", MIME"image/svg+xml"},
    𝒱::RobotNavigationVisualizer
) = robot_navigation_show(𝒱)


# TODO: Figure out saving using `FileIO`'s (?) `save` instead of `Makie.save`
# For now, you can still do: `Makie.save("my_file.pdf", robot_navigation_show(𝒱))`.
#function Base.show(io::IO, mime::MIME"image/png", 𝒱::RobotNavigationVisualizer)
#    fig = robot_navigation_show(𝒱)
#    return save(io, fig)
#end


function prettystring(s::RobotNavigationState)
    return (
        "RobotNavigationState(pose = ($(s.pose.x), $(s.pose.y), $(s.pose.θ)), "
        * "map_name = $(s.map_name), task_color = $(s.task_color))"
    )
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
    show_task_progress_belief::Bool
    text::String
end


robot_navigation_visualizer(
    𝒫::RobotNavigationPOMDP,
    step::Any;
    show_most_likely_map = false,
    show_current_state = false,
    show_task_progress_belief = false,
    text::String = ""
) = RobotNavigationVisualizer(
    𝒫, step, show_most_likely_map, show_current_state, show_task_progress_belief, text
)


function sorted_map_names(𝒱::RobotNavigationVisualizer)
    if !𝒱.show_most_likely_map
        return sort([mn for mn in keys(𝒱.𝒫.maps)])
    else
        # Determine the most likely map by sorting by the particles count.
        map_names_and_counts = Dict(mn => 0.0 for (mn, m) in 𝒱.𝒫.maps)
    
        bp = nothing
        if !𝒱.show_current_state && haskey(𝒱.step, :bp)
            bp = 𝒱.step[:bp]
        elseif haskey(𝒱.step, :b)
            bp = 𝒱.step[:b]
        end
        if bp isa AbstractParticleBelief
            for spb in particles(bp)
                map_names_and_counts[spb.map_name] += 1
            end
        end

        map_names_and_counts = sort(
            collect(map_names_and_counts),
            by = x -> string(last(x)) * "_" * string(first(x)),
            rev = true,
        )

        return [mn for (mn, c) in map_names_and_counts]
    end
end


function render_robot_action!(m::Any, 𝒱::RobotNavigationVisualizer, step::Any)
    if !haskey(step, :a)
        return
    end

    a = step[:a]

    # TODO
end


function render_robot_state_prime!(m::Any, 𝒱::RobotNavigationVisualizer, step::Any)
    if !haskey(step, :sp)
        return
    end

    sp = step[:sp]

    # Compute the robot and particle size, in *Figure*-pixel space (not image-pixel space).
    figure_pixels_per_image_pixels = (
        min(m.w, m.h)
        / min(𝒱.𝒫.maps[sp.map_name].image_width, 𝒱.𝒫.maps[sp.map_name].image_width)
    )
    marker_size = (
        (2.0 * 𝒱.𝒫.robot_radius)
        / 𝒱.𝒫.meters_per_pixel
        * figure_pixels_per_image_pixels
    )

    for (i, map_name) in enumerate(sorted_map_names(𝒱))
        map = 𝒱.𝒫.maps[map_name]
        ax = m.axes[i]

        # Render the map image.
        # NOTE: The `'` tick mark is key here. The `image` is a `Matrix`.
        # The `'` tick mark transposes the image properly to render it.
        # NOTE: The `interpolate` setting is to set it to `NEAREST` and
        # become a crisp pixel image.
        # TODO: Use step[:bp] to get an alpha-ed distribution over the maps.
        image!(
            ax, 
            (0, 𝒱.𝒫.maps[map_name].image_width * 𝒱.𝒫.meters_per_pixel),
            (0, 𝒱.𝒫.maps[map_name].image_height * 𝒱.𝒫.meters_per_pixel),
            𝒱.𝒫.maps[map_name].image',
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
                            rotation = [-θ], # NOTE: Negative for yreverse; x,y correct.
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
                rotation = [-θ], # NOTE: Negative for yreverse; x,y correct.
                color = :green,
                #alpha = 1.0,
                marker = :rtriangle, #'→',
                markersize = marker_size,
                #markerspace = :data,
            )

            scatter!(
                ax,
                [x],
                [y],
                rotation = [-θ], # NOTE: Negative for yreverse; x,y correct.
                color = :white,
                #alpha = 1.0,
                marker = '→',
                markersize = marker_size * 0.5,
                #markerspace = :data,
            )
        end
    end
end


function render_robot_observation!(m::Any, 𝒱::RobotNavigationVisualizer, step::Any)
    if !haskey(step, :o) || !haskey(step, :sp)
        return
    end

    sp = step[:sp]
    o = step[:o]

    # Compute the robot and particle size, in *Figure*-pixel space (not image-pixel space).
    figure_pixels_per_image_pixels = (
        min(m.w, m.h)
        / min(𝒱.𝒫.maps[sp.map_name].image_width, 𝒱.𝒫.maps[sp.map_name].image_width)
    )
    marker_size = (
        (2.0 * 𝒱.𝒫.robot_radius)
        / 𝒱.𝒫.meters_per_pixel
        * figure_pixels_per_image_pixels
    )

    # Figure out which map the robot is really in.
    map_index = 1
    for (i, map_name) in enumerate(sorted_map_names(𝒱))
        map = 𝒱.𝒫.maps[map_name]
        if map_name == sp.map_name
            map_index = i
            break
        end
    end

    # Render all of the observations, starting from the robot's true state.
    x, y, θ = sp.pose.x, sp.pose.y, sp.pose.θ

    for scan in o.scans
        lines!(
            m.axes[map_index],
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
            m.axes[map_index],
            [x + scan.depth * cos(θ + scan.ϕ)],
            [y + scan.depth * sin(θ + scan.ϕ)],
            rotation = [-(θ + scan.ϕ)], # NOTE: Negative for yreverse; x,y correct.
            color = c,
            #alpha = 1.0,
            marker = :circle, #'→',
            markersize = marker_size / 2.0,
            strokewidth = marker_size / 23.0,
            strokecolor = sc,
        )
    end
end


function axes_for_equally_sized_maps!(m::Any, 𝒱::RobotNavigationVisualizer)
    map_names = sorted_map_names(𝒱)

    for (i, map_name) in enumerate(map_names)
        map = 𝒱.𝒫.maps[map_name]
        fig_r = ceil(Int, i / 2)
        fig_c = (i - 1) % 2 + 1

        # NOTE: Both `Images` and `Makie` have `Axis`.
        ax = Makie.Axis(
            m.fig[fig_r, fig_c],
            yreversed = true,
            title = string(map_name), titlesize = 20,
            xlabel = "x (meters)", xlabelsize = 20,
            ylabel = "y (meters)", ylabelsize = 20,
            limits = (
                (0, 𝒱.𝒫.maps[map_name].image_width * 𝒱.𝒫.meters_per_pixel),
                (0, 𝒱.𝒫.maps[map_name].image_height * 𝒱.𝒫.meters_per_pixel),
            ),
        )
        push!(m.axes, ax)
    end

    # This Figure location is where the task heatmap axis should be placed.
    i = length(map_names) + 1
    fig_r = ceil(Int, i / 2)
    fig_c = (i - 1) % 2 + 1
    return fig_r, fig_c
end


function axes_for_most_likely_map!(m::Any, 𝒱::RobotNavigationVisualizer)
    map_names = sorted_map_names(𝒱)

    if length(map_names) == 1 && !𝒱.show_task_progress_belief
        most_likely_block_size = 1
    elseif length(map_names) - 1 + (𝒱.show_task_progress_belief ? 1 : 0) <= 4
        most_likely_block_size = 2
    elseif length(map_names) - 1 + (𝒱.show_task_progress_belief ? 1 : 0) <= 6
        most_likely_block_size = 3
    else
        most_likely_block_size = 4
    end

    # Setup the figures to highlight the most likely map.
    for (i, map_name) in enumerate(map_names)
        map = 𝒱.𝒫.maps[map_name]
        j = i - 1
        fig_r = ceil(Int, j / 2)
        fig_c = (j - 1) % 2 + 1

        if j == 0
            # NOTE: Both `Images` and `Makie` have `Axis`.
            ax = Makie.Axis(
                m.fig[1:most_likely_block_size, 1:most_likely_block_size],
                yreversed = true,
                title = string(map_name), titlesize = 20,
                xlabel = "x (meters)", xlabelsize = 20,
                ylabel = "y (meters)", ylabelsize = 20,
                limits = (
                    (0, 𝒱.𝒫.maps[map_name].image_width * 𝒱.𝒫.meters_per_pixel),
                    (0, 𝒱.𝒫.maps[map_name].image_height * 𝒱.𝒫.meters_per_pixel),
                ),
            )
            push!(m.axes, ax)
        else
            # NOTE: Both `Images` and `Makie` have `Axis`.
            ax = Makie.Axis(
                m.fig[fig_r, most_likely_block_size + fig_c],
                yreversed = true,
                title = string(map_name), titlesize = 20,
                xlabel = "x (meters)", xlabelsize = 20,
                ylabel = "y (meters)", ylabelsize = 20,
                limits = (
                    (0, 𝒱.𝒫.maps[map_name].image_width * 𝒱.𝒫.meters_per_pixel),
                    (0, 𝒱.𝒫.maps[map_name].image_height * 𝒱.𝒫.meters_per_pixel),
                ),
            )
            push!(m.axes, ax)
        end
    end

    # This Figure location is where the task heatmap axis should be placed.
    j = (length(map_names) + 1) - 1
    fig_r = ceil(Int, j / 2)
    fig_c = most_likely_block_size + (j - 1) % 2 + 1
    return fig_r, fig_c
end


function axis_and_render_task_progress_belief!(
    m::Any, 𝒱::RobotNavigationVisualizer, fig_r::Int, fig_c::Int
)
    map_names = sorted_map_names(𝒱)

    # Create the axis for rendering the task probability heatmap.
    # NOTE: Both `Images` and `Makie` have `Axis`.
    ax = Makie.Axis(
        m.fig[fig_r, fig_c],
        #yreversed = true, # NOTE: We do *not* reverse the y here.
        title = "Task Progress Belief", titlesize = 20,
        xlabel = "Task", xlabelsize = 20,
        ylabel = "Map", ylabelsize = 20,
        limits = (
            (0.5, length(TASK_COLORS) + 0.5),
            (0.5, length(map_names) + 0.5),
        ),
        xticks = (
            1:length(TASK_COLORS),
            [string(t) for t in TASK_COLORS]
        ),
        yticks = (
            1:length(map_names),
            [string(mn) for mn in reverse(map_names)]
        ),
    )
    push!(m.axes, ax)

    # Get the probabilities.
    bp = nothing
    sp = nothing

    if !𝒱.show_current_state
        if haskey(𝒱.step, :bp)
            bp = 𝒱.step[:bp]
        end
        if haskey(𝒱.step, :sp)
            sp = 𝒱.step[:sp]
        end
    elseif 𝒱.show_current_state
        if haskey(𝒱.step, :b)
            bp = 𝒱.step[:b]
        end
        if haskey(𝒱.step, :s)
            sp = 𝒱.step[:s]
        end
    end

    # Render the proability heatmap on this axis.
    if bp != nothing
        xs = 1:length(TASK_COLORS)
        ys = 1:length(map_names)
        pr = [0.0 for x in xs, y in ys]

        if bp isa AbstractParticleBelief
            num_particles = length(particles(bp))
            for spb in particles(bp)
                x = findfirst(c -> c == spb.task_color, TASK_COLORS)
                y = findfirst(mn -> mn == spb.map_name, reverse(map_names))
                pr[x, y] += 1.0 / num_particles
            end
        end

        heatmap!(ax, xs, ys, pr)
    end

    # Render the true state on this axis' heatmap.
    if sp != nothing
        x = findfirst(c -> c == sp.task_color, TASK_COLORS)
        y = findfirst(mn -> mn == sp.map_name, reverse(map_names))
        scatter!(
            ax,
            [x],
            [y],
            rotation = [pi/2],
            color = :green,
            marker = :rtriangle,
            markersize = 23
        )
    end
end


function robot_navigation_show(𝒱::RobotNavigationVisualizer)
    map_names = sorted_map_names(𝒱)

    base_figure_size_in_pixels = 500

    # Get width and height information in pixels over all maps.
    fig_width, fig_height, max_aspect_ratio = 0.0, 0.0, 1.0
    for map_name in map_names
        map = 𝒱.𝒫.maps[map_name]
        fig_width = max(fig_width, map.image_width)
        fig_height = max(fig_height, map.image_height)
        max_aspect_ratio = max(
            max_aspect_ratio,
            map.image_width / map.image_height
        )
    end
    fig_width = base_figure_size_in_pixels * max_aspect_ratio
    fig_height = base_figure_size_in_pixels

    # Get how many figure blocks to reserve (a square).
    # Dividing this by 2 reserves enough for half the figure.
    num_blocks = 0
    if !𝒱.show_most_likely_map
        num_blocks = (
            length(map_names)
            + (𝒱.show_task_progress_belief ? 1 : 0)
        )
    else
        num_blocks = ceil(
            Int, (
                (length(map_names) - 1)
                + (𝒱.show_task_progress_belief ? 1 : 0)
            )
        )
    end
    num_block_rows = ceil(Int, num_blocks / 2.0)
    num_block_cols = (num_blocks == 1 ? 1 : 2)

    # NOTE: The labels on the x-axis are 1 *per plot*. Conversely,
    # on the y-axis, there is 1 label and a title. This extra bit
    # causes the images to be slightly stretched horizontally.
    # Optionally, play around with these numbers.
    extra_x_padding_in_pixels = 0
    extra_y_padding_in_pixels = 0

    if !𝒱.show_most_likely_map
        w = (fig_width + extra_x_padding_in_pixels) * num_block_cols
        h = (fig_height + extra_y_padding_in_pixels) * num_block_rows
    else
        if num_blocks == 0
            w = fig_width + extra_x_padding_in_pixels
        elseif num_blocks == 1
            w = (
                fig_width + extra_x_padding_in_pixels
                + (fig_width + extra_x_padding_in_pixels) * num_block_cols
            )
        else
            w = (
                fig_width * 2.0 + extra_x_padding_in_pixels
                + (fig_width + extra_x_padding_in_pixels) * num_block_cols
            )
        end
        h = (fig_height + extra_y_padding_in_pixels) * num_block_rows
    end

    fig = Figure(size = (w, h))
    axes = Makie.Axis[]
    m = (fig = fig, axes = axes, w = w, h = h)

    if !𝒱.show_most_likely_map
        fig_r, fig_c = axes_for_equally_sized_maps!(m, 𝒱)
    else
        fig_r, fig_c = axes_for_most_likely_map!(m, 𝒱)
    end

    # Render the task heatmap below all these maps.
    if 𝒱.show_task_progress_belief
        axis_and_render_task_progress_belief!(m, 𝒱, fig_r, fig_c)
    end

    # Normally, render the resulting successor state, belief, observation,
    # and so on. Otherwise, show the current state, action, and so on.
    if !𝒱.show_current_state
        render_robot_action!(m, 𝒱, 𝒱.step)
        render_robot_state_prime!(m, 𝒱, 𝒱.step)
        render_robot_observation!(m, 𝒱, 𝒱.step)
    elseif haskey(𝒱.step, :s) && haskey(𝒱.step, :b)
        step′ = Dict(
            :sp => deepcopy(𝒱.step[:s]),
            :bp => deepcopy(𝒱.step[:b]),
            :o => deepcopy(𝒱.step[:o]),
        )
        render_robot_state_prime!(m, 𝒱, step′)
        render_robot_action!(m, 𝒱, step′)
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


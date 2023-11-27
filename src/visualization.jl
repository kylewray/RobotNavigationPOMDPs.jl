function prettystring(s::RobotNavigationState)
    return "RobotNavigationState(pose = ($(s.pose.x), $(s.pose.y), $(s.pose.θ)), map_name = $(s.map_name), task_color = $(s.task_color))"
end

Base.show(io::IO, s::RobotNavigationState) = println(io, prettystring(s))
Base.show(io::IO, ::MIME"text/plain", s::RobotNavigationState) = println(io, prettystring(s))


function prettystring(a::RobotNavigationAction)
    return "RobotNavigationAction(desired_move = $(a.desired_move), desired_θ = $(a.desired_θ))"
end

Base.show(io::IO, a::RobotNavigationAction) = println(io, prettystring(a))
Base.show(io::IO, ::MIME"text/plain", a::RobotNavigationAction) = println(io, prettystring(a))


function prettystring(o::RobotNavigationObservation)
    result = "RobotNavigationObservation(depths = [\n"
    for i in 1:length(o.scans)
        result *= "    depth = $(o.scans[i].depth), color = $(o.scans[i].color)\n"
    end
    result *= "]"
    return result
end

Base.show(io::IO, o::RobotNavigationObservation) = println(io, prettystring(o))
Base.show(io::IO, ::MIME"text/plain", o::RobotNavigationObservation) = println(io, prettystring(o))


function transform_coordinates(𝒫::RobotNavigationPOMDP, s::RobotNavigationState, point::SVec2)
    x, y = point

    image = 𝒫.maps[s.map_name].image
    height = round(Int, size(image, 1))
    width = round(Int, size(image, 2))

    yImage = floor(Int, y / 𝒫.meters_per_pixel) + 1
    xImage = floor(Int, x / 𝒫.meters_per_pixel) + 1

    return xImage, yImage
end


struct RobotNavigationVisualizer
    𝒫::RobotNavigationPOMDP
    step::Any
    text::String
end

render_robot_navigation(𝒫::RobotNavigationPOMDP, step::Any; text::String = "") = RobotNavigationVisualizer(𝒫, step, text)

function render_robot_navigation(ctx::CairoContext, 𝒫::RobotNavigationPOMDP, step::Any)
    s = step[:sp]
    
    # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    # TODO: Ok, so this equation is 0.25 m / 4.0 m/p = 1 pixel... This is bad.
    # The reason is that I am drawing the entire image, which is 10 by 10 pixels.
    # But then I scale that image by width / image.width and height / image.height.
    # So, if you want to show the cricle of the robot, you need to scale both the x and y
    # of the "arc" circle below, as well as the "radius" term in the "arc" function!
    # Figure this out. (I tested manually placing a circle, and it worked). Good luck!
    # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

    radius = 𝒫.robot_radius / 𝒫.meters_per_pixel
    width = ctx.surface.width
    height = ctx.surface.height

    # Render a gray background.
    Cairo.save(ctx)
    set_source_rgb(ctx, 0.8, 0.8, 0.8)
    rectangle(ctx, 0.0, 0.0, width, height)
    fill(ctx)
    Cairo.restore(ctx)

    # Render the map image.
    # NOTE: "Cairo." is required due to name conflict with Images.
    # NOTE: "Cairo." is required also for scale for conflict with Distributions.
    # TODO: Use step[:bp] to get an alpha-ed distribution over the maps.
    Cairo.save(ctx)
    image = read_from_png(𝒫.maps[s.map_name].absolute_path)
    Cairo.scale(ctx, width / image.width, height / image.height)
    set_source_surface(ctx, image, 0, 0)
    pattern_set_filter(get_source(ctx), Cairo.FILTER_NEAREST)
    paint(ctx)
    Cairo.restore(ctx)

    # Render the particle beliefs (if any).
    Cairo.save(ctx)
    if haskey(step, :bp)
        bp = step[:bp]
        if bp isa AbstractParticleBelief
            for s in particles(bp)
                # NOTE: The heading p.θ is not rendered.
                x, y = transform_coordinates(𝒫, s, SVec2(s.pose.x, s.pose.y))
                arc(ctx, x, y, radius, 0, 2.0 * float(π))
                set_source_rgba(ctx, 0.6, 0.6, 1.0, 0.3)
                fill(ctx)
            end
        end
    end
    Cairo.restore(ctx)

    # Render the robot base circle.
    Cairo.save(ctx)
    x, y = transform_coordinates(𝒫, s, SVec2(s.pose.x, s.pose.y))
    arc(ctx, x, y, radius, 0, 2.0 * float(π))
    set_source_rgb(ctx, 0.6, 0.6, 1.0)
    fill(ctx)
    Cairo.restore(ctx)

    # Render the robot's heading by a short line segment.
    Cairo.save(ctx)
    move_to(ctx, x, y)
    point = SVec2(
        s.pose.x + 𝒫.meters_per_pixel * cos(s.pose.θ),
        s.pose.y + 𝒫.meters_per_pixel * sin(s.pose.θ)
    )
    x′, y′ = transform_coordinates(𝒫, s, point)
    line_to(ctx, x′, y′)
    set_source_rgb(ctx, 0, 0, 0)
    stroke(ctx)
    Cairo.restore(ctx)

    println("spx, spy = ", s.pose.x, ", ", s.pose.y)
    println("x, y = ", x, ", ", y)
    println("x′, y′ = ", x′, ", ", y′)

    return ctx
end

function Base.show(io::IO, mime::Union{MIME"text/html", MIME"image/svg+xml"}, 𝒱::RobotNavigationVisualizer)
    c = CairoSVGSurface(io, 800, 600)
    ctx = CairoContext(c)
    render_robot_navigation(ctx, 𝒱.𝒫, 𝒱.step)
    return finish(c)
end

function Base.show(io::IO, mime::MIME"image/png", 𝒱::RobotNavigationVisualizer)
    c = CairoRGBSurface(800, 600)
    ctx = CairoContext(c)
    render_robot_navigation(ctx, 𝒱.𝒫, 𝒱.step)
    return write_to_png(c, io)
end

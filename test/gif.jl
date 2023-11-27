using RobotNavigationPOMDPs

using POMDPs
using POMDPModels
using POMDPTools
using POMDPSimulators
using ParticleFilters

using Cairo
using Random
#using Plots
#using Reel
#using ProgressMeter


𝒫 = RobotNavigationPOMDP()
policy = RandomPolicy(𝒫)

rng = MersenneTwister(23)

hr = HistoryRecorder(max_steps = 30, rng = rng)
filter = SIRParticleFilter(𝒫, 200, rng = rng)
history = POMDPs.simulate(hr, 𝒫, policy, filter)

#for i in 1:n_steps(history)
#    render_robot_navigation(𝒫
#end

step = (
    t = 1,
    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 3.0 * 𝒫.meters_per_pixel,
            y = 8.0 * 𝒫.meters_per_pixel,
            θ = float(π) / 4.0
        ),
        map_name = :map,
        #task_color = RobotNavigationColor.MAGENTA
    ),
    a = RobotNavigationAction(),
    r = -1.0,
    sp = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 3.0 * 𝒫.meters_per_pixel,
            y = 8.0 * 𝒫.meters_per_pixel,
            θ = float(π) / 4.0
        ),
        map_name = :map,
        #task_color = RobotNavigationColor.MAGENTA
    ),
    o = RobotNavigationObservation(),
)
𝒱 = render_robot_navigation(𝒫, step)

c = CairoRGBSurface(1024, 768)
ctx = CairoContext(c)
render_robot_navigation(ctx, 𝒱.𝒫, 𝒱.step)
write_to_png(c, "robot_navigation_example.gif")


# Version 2: Try this after you get above working.
#frames = []
#for i in 1:n_steps(history)
#    c = CairoRGBSurface(800, 600)
#    ctx = CairoContext(c)
#    render_robot_navigation(ctx, 𝒱.𝒫, 𝒱.step)
#    push!(frames, ctx)
#end
#
#filename = string("robot_navigation_example.gif")
#FileIO.save(filename, frames)


# Version 1: This does not work.
#frames = Frames(MIME("image/png"), fps=2)
#
#gr()
#@showprogress "Creating gif..." for i in 1:n_steps(hist)
#    push!(frames, plot(𝒫, view(hist, 1:i)))
#end
#
#filename = string("robot_navigation_example.gif")
#write(filename, frames)
#println(String(pwd()) * "/" * filename)


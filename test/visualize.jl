using RobotNavigationPOMDPs

using POMDPs
using POMDPModels
using POMDPTools
using POMDPSimulators
using ParticleFilters

using CairoMakie
using Random
#using ProgressMeter


function visualize()
    𝒫 = RobotNavigationPOMDP()
    policy = RandomPolicy(𝒫)

    rng = MersenneTwister(23)

    hr = HistoryRecorder(max_steps = 5, rng = rng)
    filter = SIRParticleFilter(𝒫, 200, rng = rng)
    history = POMDPs.simulate(hr, 𝒫, policy, filter)

    for i in 1:length(history)
        step = history[i]

        𝒱 = robot_navigation_visualizer(𝒫, step)
        fig = robot_navigation_show(𝒱)
        save("visualize_$i.pdf", fig)
    end

end


visualize()

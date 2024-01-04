module RobotNavigationPOMDPs

using POMDPs
using POMDPTools
using POMDPModelTools       # Deterministic, ImplicitDistribution
using ParticleFilters       # AbstractParticleBelief

using IterTools             # product
using Combinatorics         # powerset
using Parameters            # @with_kw
using Printf                # @printf
using Random                # rand
using Distributions         # Normal

using FileIO                # joinpath
import Base: rand, show     # rand(*, Distributions.Normal(.)), show

using Images                # load
using StaticArrays          # SVector (use for vector performance)
using Makie                 # Visualization...
using CairoMakie            # Visualization...


const SVec2 = SVector{2, Float64}
const SVec3 = SVector{3, Float64}


function loop_angle(θ)
    fπ = float(π)
    while θ > fπ
        θ -= 2.0 * fπ
    end
    while θ < -fπ
        θ += 2.0 * fπ
    end
    return θ
end


function turn_to_angle(first_θ, second_θ)
    fπ = float(π)
    θ₁ = loop_angle(first_θ)
    θ₂ = loop_angle(second_θ)

    if abs(θ₁ - θ₂) > fπ
        if θ₂ > θ₁
            θ₁ += 2.0 * fπ
        else
            θ₂ += 2.0 * fπ
        end
    end

    return θ₂ - θ₁
end


function difference_of_angles(first_θ, second_θ)
    return abs(turn_to_angle(first_θ, second_θ))
end


function permutations_with_replacement(X)
    return vec(collect(IterTools.product(X...)))
end


include("model.jl")
export
    RobotNavigationPOMDP,
    RobotNavigationMap,
    RobotNavigationColor,
    RobotNavigationPose,
    RobotNavigationState,
    RobotNavigationStates,
    RobotNavigationAction,
    RobotNavigationActions,
    RobotNavigationObservation,
    RobotNavigationObservations

include("visualization.jl")
export
    RobotNavigationVisualizer,
    robot_navigation_visualizer,
    robot_navigation_show

include("benchmarks.jl")
export
    RobotNavigationPOMDPSimpleTest,
    RobotNavigationPOMDPSimpleLocalization,
    RobotNavigationPOMDPSimpleSLAM,
    RobotNavigationPOMDPMineSLAM

end # module RobotNavigationPOMDPs

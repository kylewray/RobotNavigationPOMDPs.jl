module RobotNavigationPOMDPs

using POMDPs
using POMDPTools
using POMDPModelTools   # Deterministic, ImplicitDistribution

using IterTools         # product
using Combinatorics     # powerset
using Parameters        # @with_kw
using Printf            # @printf
using Random            # rand
using Distributions     # Normal

using FileIO            # joinpath
import Base: rand       # rand(*, Distributions.Normal(.))

using Images            # load
using StaticArrays      # SVector (use for vector performance)
using Cairo             # CairoContext, image, and so on...


const SVec2 = SVector{2, Float64}


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
    render_robot_navigation

end # module RobotNavigationPOMDPs

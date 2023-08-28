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


function loop_angle(θ)
    fπ = float(π)
    if θ > fπ
        θ -= 2.0 * fπ
    elseif θ < -fπ
        θ += 2.0 * fπ
    end
    return θ
end


function turn_to_angle(current_θ, desired_θ)
    fπ = float(π)

    if abs(current_θ - desired_θ) > fπ
        if desired_θ > current_θ
            current_θ += 2.0 * fπ
        else
            desired_θ += 2.0 * fπ
        end
    end

    return desired_θ - current_θ
end


function permutations_with_replacement(X)
    return vec(collect(IterTools.product(X...)))
end


include("robot_navigation_pomdp.jl")
export
    RobotNavigationPOMDP,
    RobotNavigationPose,
    RobotNavigationState,
    RobotNavigationStates,
    RobotNavigationAction,
    RobotNavigationActions,
    RobotNavigationObservation,
    RobotNavigationObservations ###,
    #RobotNavigationBeliefUpdater,
    #render

end # module RobotNavigationPOMDPs

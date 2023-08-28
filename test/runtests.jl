using RobotNavigationPOMDPs

using POMDPs
using Printf    # @printf
using Random    # MersenneTwister, rand
using Test      # @testset, @test

@testset "LoopAngle" begin
    fπ = float(π)
    @test RobotNavigationPOMDPs.loop_angle(0.0) == 0.0
    @test RobotNavigationPOMDPs.loop_angle(fπ) == fπ
    @test RobotNavigationPOMDPs.loop_angle(-fπ) == -fπ
    @test RobotNavigationPOMDPs.loop_angle(fπ + 1.0) == -fπ + 1.0
    @test RobotNavigationPOMDPs.loop_angle(-fπ - 1.0) == fπ - 1.0
end

@testset "TurnToAngle" begin
    fπ = float(π)
    @test RobotNavigationPOMDPs.turn_to_angle(0.0, 0.0) == 0.0
    @test RobotNavigationPOMDPs.turn_to_angle(fπ, 0.0) == -fπ
    @test RobotNavigationPOMDPs.turn_to_angle(0.0, fπ) == fπ
    @test RobotNavigationPOMDPs.turn_to_angle(3.0 / 2.0 * fπ, 0.0) == fπ / 2.0
    @test RobotNavigationPOMDPs.turn_to_angle(0.0, 3.0 / 2.0 * fπ) == -fπ / 2.0
    @test RobotNavigationPOMDPs.turn_to_angle(fπ / 4.0, -fπ / 4.0) == -fπ / 2.0
    @test RobotNavigationPOMDPs.turn_to_angle(-fπ / 4.0, fπ / 4.0) == fπ / 2.0
end

@testset "PermutationsWithReplacement" begin
    X = [[:a, :b], [:x, :y, :z]]
    P = RobotNavigationPOMDPs.permutations_with_replacement(X)
    @test P == [(:a, :x), (:b, :x), (:a, :y), (:b, :y), (:a, :z), (:b, :z)]

    Y = [1, 2]
    P = RobotNavigationPOMDPs.permutations_with_replacement(Y)
    @test P == [(1, 2)]

    Z = []
    P = RobotNavigationPOMDPs.permutations_with_replacement(Z)
    @test Z == []
end

@testset "DeterministicRobotNavigationPOMDP" begin
    @printf("Testing default deterministic RobotNavigationPOMDP...\n")
    𝒫 = RobotNavigationPOMDP()
    @printf("Created POMDP!\n")

    𝒮 = POMDPs.states(𝒫)
    @printf("Number of States:       %i\n", length(𝒮))
    𝒜 = POMDPs.actions(𝒫)
    @printf("Number of Actions:      %i\n", length(𝒜))
    𝒪 = POMDPs.observations(𝒫)
    @printf("Number of Observations: %i\n", length(𝒪))

    @test length(𝒮) == 400
    @test length(𝒜) == 8
    @test length(𝒪) == 16

    @printf("Testing custom deterministic RobotNavigationPOMDP...\n")
    𝒫 = RobotNavigationPOMDP(map_names = ["one", "two"],
                             size_of_map = Dict("one" => (width = 6, height = 4),
                                                "two" => (width = 4, height = 2),
                                               ),
                             meters_per_pixel = 0.5,
                             num_determinized_orientations = 4,
                             room_names_for_map = Dict("one" => ["r1", "r2"], "two" => ["r3"]),
                             task_names_for_map = Dict("one" => ["t1", "t2"], "two" => ["t3", "t4", "t5"]),
                             max_xy_speed = 1.0,
                             max_θ_speed = float(π) / 4.0,
                             num_depths = 4,
                             max_depth_range = 1.0,
                             num_objects = 1,
                            )
    @printf("Created POMDP!\n")

    𝒮 = POMDPs.states(𝒫)
    @printf("Number of States:       %i\n", length(𝒮))
    𝒜 = POMDPs.actions(𝒫)
    @printf("Number of Actions:      %i\n", length(𝒜))
    𝒪 = POMDPs.observations(𝒫)
    @printf("Number of Observations: %i\n", length(𝒪))

    @test length(𝒮) == 72
    @test length(𝒜) == 8
    @test length(𝒪) == 32
end

@testset "RobotNavigationPOMDP" begin
    @printf("Testing RobotNavigationPOMDP T, O, and R...\n")
    𝒫 = RobotNavigationPOMDP()

    rng = MersenneTwister(23)

    𝒮 = POMDPs.states(𝒫)
    𝒜 = POMDPs.actions(𝒫)
    𝒪 = POMDPs.observations(𝒫)

    s = RobotNavigationState(pose = RobotNavigationPose(x = 1, y = 1, θ = 0.0), map_name = "map", task_name = "task")
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    s′ = rand(rng, transition(𝒫, s, a))
    @test s′.pose.x == s.pose.x && s′.pose.y == s.pose.y && s′.pose.θ == s.pose.θ
    s′ = rand(rng, transition(𝒫, s, a))
    @test s′.pose.x == s.pose.x && s′.pose.y == s.pose.y && s′.pose.θ == s.pose.θ

    s = RobotNavigationState(pose = RobotNavigationPose(x = 1, y = 1, θ = 0.0), map_name = "map", task_name = "task")
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ + float(π))
    println(dump(rand(rng, transition(𝒫, s, a))))
    println(dump(rand(rng, transition(𝒫, s, a))))
    println(dump(rand(rng, transition(𝒫, s, a))))
    println(dump(rand(rng, transition(𝒫, s, a))))
    println(dump(rand(rng, transition(𝒫, s, a))))
    #@test rand(rng, transition(𝒫, s, a)) == s
    #
    # TODO: Working! Now get many samples to confirm it has a valid mean of the desired heading (change float(π) though...)
    # TODO: Do the same for the movement action.
    # TODO: Do the same for both.

    # TODO: Create observation function.

    # TODO: Create reward function.
end


using RobotNavigationPOMDPs

using POMDPs
using POMDPModels
using POMDPTools
using POMCPOW

using Printf        # @printf
using Random        # MersenneTwister, rand
using Test          # @testset, @test

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
    @test length(𝒪) == 1000

    @printf("Testing custom deterministic RobotNavigationPOMDP...\n")
    𝒫 = RobotNavigationPOMDP(
        maps = Dict(
            :one => RobotNavigationMap(:one, "default.png"),
            :two => RobotNavigationMap(:two, "default.png"),
        ),
        size_of_map = Dict(
            :one => (width = 6, height = 4),
            :two => (width = 4, height = 2),
        ),
        meters_per_pixel = 0.5,
        num_determinized_orientations = 4,
        max_tasks_per_map = 3,
        task_color_for_map = Dict(
            :one => [
                RobotNavigationPOMDPs.RED,
                RobotNavigationPOMDPs.MAGENTA
            ],
            :two => [
                RobotNavigationPOMDPs.GREEN,
                RobotNavigationPOMDPs.BLUE,
                RobotNavigationPOMDPs.MAGENTA
            ]
        ),
        robot_radius = 0.25,
        move_xy_max_speed = 1.0,
        move_θ_max_speed = float(π) / 4.0,
        move_xy_variance = 0.1,
        move_θ_variance = 0.1,
        num_scans = 3,
        scan_field_of_view = float(π) / 2.0,
        scan_range = 3.0,
        scan_depth_variance = 0.1,
        scan_color_observation_probability = 0.9
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
    @test length(𝒪) == 1000
end

@testset "RobotNavigationPOMDPTransitions" begin
    @printf("Testing RobotNavigationPOMDP Transitions...\n")
    𝒫 = RobotNavigationPOMDP()

    rng = MersenneTwister(23)

    s = RobotNavigationState(
        pose = RobotNavigationPose(x = 1, y = 1, θ = 0.0),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    for i in 1:10
        s′ = rand(rng, transition(𝒫, s, a))
        @test (
            s′.pose.x == s.pose.x && s′.pose.y == s.pose.y && s′.pose.θ == s.pose.θ
            && s′.map_name == s.map_name && s′.task_color == s.task_color
        )
        @printf(
            "    Confirmed that s′.pose.θ == s.pose.θ: %.2f =?= %.2f\n",
            s′.pose.θ, s.pose.θ
        )
    end

    s = RobotNavigationState(
        pose = RobotNavigationPose(x = 1, y = 1, θ = 0.0),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ + float(π))
    for i in 1:10
        s′ = rand(rng, transition(𝒫, s, a))
        @test (
            s′.pose.x == s.pose.x && s′.pose.y == s.pose.y
            && s′.pose.θ >= s.pose.θ # NOTE: With low probability this will be bad.
            && s′.map_name == s.map_name && s′.task_color == s.task_color
        )
        @printf(
            "    Confirmed that s′.pose.θ >= s.pose.θ: %.2f >?= %.2f\n",
            s′.pose.θ, s.pose.θ
        )
    end

    s = RobotNavigationState(
        pose = RobotNavigationPose(x = 1, y = 1, θ = float(π)),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ - float(π))
    for i in 1:10
        s′ = rand(rng, transition(𝒫, s, a))
        @test (
            s′.pose.x == s.pose.x && s′.pose.y == s.pose.y
            && s′.pose.θ <= s.pose.θ # NOTE: With low probability this will be bad.
            && s′.map_name == s.map_name && s′.task_color == s.task_color
        )
        @printf(
            "    Confirmed that s′.pose.θ <= s.pose.θ: %.2f <?= %.2f\n",
            s′.pose.θ, s.pose.θ
        )
    end

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            # A white pixel in "default.png".
            x = 3.0 * 𝒫.meters_per_pixel,
            y = 8.0 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = true, desired_θ = s.pose.θ)
    for i in 1:10
        s′ = rand(rng, transition(𝒫, s, a))
        @test (
            s′.pose.x >= s.pose.x # NOTE: With low probability this will be bad.
            && abs(s′.pose.y - s.pose.y) < 0.1 # NOTE: With low probability this will be bad.
            && s′.pose.θ == s.pose.θ
            && s′.map_name == s.map_name && s′.task_color == s.task_color
        )
        @printf(
            "    Confirmed that s′.pose.x >= s.pose.x: %.2f >?= %.2f\n",
            s′.pose.x, s.pose.x
        )
    end

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            # A black pixel minus robot size in "default.png".
            x = 4.0 * 𝒫.meters_per_pixel - 𝒫.robot_radius,
            y = 6.0 * 𝒫.meters_per_pixel - 𝒫.robot_radius, 
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = true, desired_θ = s.pose.θ)
    for i in 1:10
        s′ = rand(rng, transition(𝒫, s, a))
        @test (
            s′.pose.x == s.pose.x # NOTE: With low probability this will be bad.
            && abs(s′.pose.y - s.pose.y) < 0.1 # NOTE: With low probability this will be bad.
            && s′.pose.θ == s.pose.θ
            && s′.map_name == s.map_name && s′.task_color == s.task_color
        )
        @printf(
            "    Confirmed that s′.pose.x >= s.pose.x: %.2f >?= %.2f\n",
            s′.pose.x, s.pose.x
        )
    end

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            # A black pixel minus robot size minus two steps in "default.png".
            x = 4.0 * 𝒫.meters_per_pixel - 𝒫.robot_radius - 𝒫.move_xy_max_speed * 0.5 * 𝒫.meters_per_pixel,
            y = 6.0 * 𝒫.meters_per_pixel - 𝒫.robot_radius - 𝒫.move_xy_max_speed * 0.5 * 𝒫.meters_per_pixel, 
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = true, desired_θ = s.pose.θ)
    for i in 1:10
        s′ = rand(rng, transition(𝒫, s, a))
        @test (
            s′.pose.x >= s.pose.x # NOTE: With low probability this will be bad.
            && abs(s′.pose.y - s.pose.y) < 0.1 # NOTE: With low probability this will be bad.
            && s′.pose.θ == s.pose.θ
            && s′.map_name == s.map_name && s′.task_color == s.task_color
        )
        @printf(
            "    Confirmed that s′.pose.x >= s.pose.x: %.2f >?= %.2f\n",
            s′.pose.x, s.pose.x
        )
    end

    for j in 1:10
        s = RobotNavigationState(
            pose = RobotNavigationPose(
                # A black pixel minus robot size minus three steps in "default.png".
                x = 7.0 * 𝒫.meters_per_pixel - 𝒫.robot_radius - 𝒫.move_xy_max_speed * 0.75 * 𝒫.meters_per_pixel,
                y = 5.0 * 𝒫.meters_per_pixel - 𝒫.robot_radius - 𝒫.move_xy_max_speed * 0.75 * 𝒫.meters_per_pixel, 
                θ = float(π)
            ),
            map_name = :map,
            task_color = RobotNavigationPOMDPs.MAGENTA
        )
        a = RobotNavigationAction(desired_move = true, desired_θ = -float(π) / 2.0)
        s′ = deepcopy(s)
        num_iterations = 0
        for i in 1:10
            prev_x, prev_y = s′.pose.x, s′.pose.y
            s′ = rand(rng, transition(𝒫, deepcopy(s′), a))
            if s′.pose.x != prev_x || s′.pose.y != prev_y
                num_iterations += 1
            end
        end
        @test (
            s′.pose.x != s.pose.x && s′.pose.y != s.pose.y && s′.pose.θ != s.pose.θ
            && s′.map_name == s.map_name && s′.task_color == s.task_color
            && num_iterations > 0 && num_iterations < 10 && num_iterations != 10
        )
        @printf("    Confirmed that robot moved %i iterations ", num_iterations)
        @printf(
            "from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f).\n",
            s.pose.x, s.pose.y, s.pose.θ, s′.pose.x, s′.pose.y, s′.pose.θ
        )
    end
end

@testset "RobotNavigationPOMDPObservations" begin
    @printf("Testing RobotNavigationPOMDP Observations...\n")
    𝒫 = RobotNavigationPOMDP(
        num_scans = 10
    )

    rng = MersenneTwister(23)

    a = RobotNavigationAction(desired_move = false, desired_θ = 0.0)
    s′ = RobotNavigationState(
        pose = RobotNavigationPose(
            # This (7, 4.5) coordinate is just far enough away from
            # the wall to give two white side and all black middle
            # detections in the scans.
            x = 7.0 * 𝒫.meters_per_pixel,
            y = 4.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    for i in 1:30
        o = rand(rng, observation(𝒫, a, s′))
        for oi in o.scans
            @test (
                oi.depth >= 2.0 * 𝒫.meters_per_pixel
                && oi.depth <= 3.5 * 𝒫.meters_per_pixel
            )
        end

        num_freespace_detections = 0
        for j in 1:𝒫.num_scans
            if o.scans[j].color == RobotNavigationPOMDPs.WHITE
                num_freespace_detections += 1
            end
        end
        @test num_freespace_detections <= 2

        num_wall_detections = 0
        for j in 1:𝒫.num_scans
            if o.scans[j].color == RobotNavigationPOMDPs.BLACK
                num_wall_detections += 1
            end
        end
        @test num_wall_detections >= 6

        @printf(
            "    Confirmed that there are %i of %i scans, ",
            length(o.scans), 𝒫.num_scans
        )
        @printf(
            "with %i of %i wall detections (min of 6) ",
            num_wall_detections, 𝒫.num_scans
        )
        @printf(
            "and %i of %i freespace detections (max of 2).\n",
            num_freespace_detections, 𝒫.num_scans
        )
    end
end

@testset "RobotNavigationPOMDPRewards" begin
    @printf("Testing RobotNavigationPOMDP Rewards...\n")
    𝒫 = RobotNavigationPOMDP()

    rng = MersenneTwister(23)

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 7.5 * 𝒫.meters_per_pixel,
            y = 2.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == -1.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 5.5 * 𝒫.meters_per_pixel,
            y = 2.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == 0.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 2.5 * 𝒫.meters_per_pixel,
            y = 2.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == -1.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 2.5 * 𝒫.meters_per_pixel,
            y = 2.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.GREEN
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == -1.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 2.5 * 𝒫.meters_per_pixel,
            y = 2.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.RED
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == 0.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 4.5 * 𝒫.meters_per_pixel,
            y = 4.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.MAGENTA
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == -1.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 4.5 * 𝒫.meters_per_pixel,
            y = 4.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.RED
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == -1.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )

    s = RobotNavigationState(
        pose = RobotNavigationPose(
            x = 4.5 * 𝒫.meters_per_pixel,
            y = 4.5 * 𝒫.meters_per_pixel,
            θ = 0.0
        ),
        map_name = :map,
        task_color = RobotNavigationPOMDPs.GREEN
    )
    a = RobotNavigationAction(desired_move = false, desired_θ = s.pose.θ)
    r = reward(𝒫, s, a)
    @test r == 0.0
    @printf(
        "    Confirmed at s.pose = (%.2f, %.2f, %.2f) and task %s, the reward is %.2f.\n",
        s.pose.x, s.pose.y, s.pose.θ, string(s.task_color), r
    )
end

@testset "RobotNavigationPOMDPInitialState" begin
    @printf("Testing RobotNavigationPOMDP Initial State...\n")
    𝒫 = RobotNavigationPOMDP()

    rng = MersenneTwister(23)

    for i in 1:30
        s = rand(rng, initialstate(𝒫))
        py = floor(Int, s.pose.y / 𝒫.meters_per_pixel) + 1
        px = floor(Int, s.pose.x / 𝒫.meters_per_pixel) + 1

        @test px == 2 || px == 9
        @test py == 2 || py == 9
        @test s.pose.θ >= 0.0 && s.pose.θ < 2.0 * float(π)

        @printf(
            "    Confirmed s.pose(%.2f, %.2f, %.2f) is on cyan pixels (%i, %i) in map image.\n",
            s.pose.x, s.pose.y, s.pose.θ, px, py
        )
    end
end

@testset "RobotNavigationPOMDPSolverPOMCPOW" begin
    @printf("Testing RobotNavigationPOMDP Solver POMCPOW...\n")

    solver = POMCPOWSolver(criterion = MaxUCB(20.0), max_depth = 7) # max_depth = 10
    𝒫 = RobotNavigationPOMDP()
    planner = solve(solver, 𝒫)

    hr = HistoryRecorder(max_steps = 100) # max_steps = 100
    pomcpow_history = simulate(hr, 𝒫, planner)
    for (s, b, a, r, sp, o) in pomcpow_history
        @show s, a, r, sp
    end

    random_policy_history = simulate(hr, 𝒫, RandomPolicy(𝒫))
    println("Cumulative Discounted Reward (for 1 simulation)")
    println("    Random: $(discounted_reward(random_policy_history))")
    println("    POMCPOW: $(discounted_reward(pomcpow_history))")
end

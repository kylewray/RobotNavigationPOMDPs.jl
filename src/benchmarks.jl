using RobotNavigationPOMDPs


function RobotNavigationPOMDPSimpleTest()
    return RobotNavigationPOMDP(
        maps = Dict(
            :m => RobotNavigationMap(:m, "simple_test.png")
        ),
        size_of_map = Dict(
            :m => (width = 10, height = 10)
        ),
        meters_per_pixel = 1.0,
        task_color_for_map = Dict(
            :m => [MAGENTA]
        ),
    )
end


function RobotNavigationPOMDPSimpleLocalization()
    return RobotNavigationPOMDP(
        maps = Dict(
            :m => RobotNavigationMap(:m, "simple_localization.png")
        ),
        size_of_map = Dict(
            :m => (width = 100, height = 100)
        ),
        meters_per_pixel = 0.1,
        task_color_for_map = Dict(
            :m => [MAGENTA]
        ),
    )
end


function RobotNavigationPOMDPSimpleSLAM()
    return RobotNavigationPOMDP(
        maps = Dict(
            :m1 => RobotNavigationMap(:m1, "simple_slam_1.png"),
            :m2 => RobotNavigationMap(:m2, "simple_slam_2.png"),
            :m3 => RobotNavigationMap(:m3, "simple_slam_3.png"),
            :m4 => RobotNavigationMap(:m4, "simple_slam_4.png"),
        ),
        size_of_map = Dict(
            :m1 => (width = 100, height = 100),
            :m2 => (width = 100, height = 100),
            :m3 => (width = 100, height = 100),
            :m4 => (width = 100, height = 100),
        ),
        meters_per_pixel = 0.25,
        task_color_for_map = Dict(
            :m1 => [MAGENTA],
            :m2 => [MAGENTA],
            :m3 => [MAGENTA],
            :m4 => [MAGENTA],
        ),
    )
end


function RobotNavigationPOMDPMineSLAM()
    return RobotNavigationPOMDP(
        maps = Dict(
            :m1 => RobotNavigationMap(:m1, "mine_slam_1.png"),
            :m2 => RobotNavigationMap(:m2, "mine_slam_2.png"),
            :m3 => RobotNavigationMap(:m3, "mine_slam_3.png"),
            :m4 => RobotNavigationMap(:m4, "mine_slam_4.png"),
            :m5 => RobotNavigationMap(:m5, "mine_slam_5.png"),
            :m6 => RobotNavigationMap(:m6, "mine_slam_6.png"),
            :m7 => RobotNavigationMap(:m7, "mine_slam_7.png"),
            :m8 => RobotNavigationMap(:m8, "mine_slam_8.png"),
        ),
        size_of_map = Dict(
            :m1 => (width = 128, height = 95),
            :m2 => (width = 128, height = 95),
            :m3 => (width = 128, height = 95),
            :m4 => (width = 128, height = 95),
            :m5 => (width = 128, height = 95),
            :m6 => (width = 128, height = 95),
            :m7 => (width = 128, height = 95),
            :m8 => (width = 128, height = 95),
        ),
        meters_per_pixel = 0.25,
        task_color_for_map = Dict(
            :m1 => [MAGENTA],
            :m2 => [MAGENTA],
            :m3 => [MAGENTA],
            :m4 => [MAGENTA],
            :m5 => [MAGENTA],
            :m6 => [MAGENTA],
            :m7 => [MAGENTA],
            :m8 => [MAGENTA],
        ),
    )
end

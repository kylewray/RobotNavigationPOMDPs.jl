using RobotNavigationPOMDPs


function RobotNavigationPOMDPToyDefault()
    return RobotNavigationPOMDP(
        maps = Dict(
            :toy_default => RobotNavigationMap(
                :toy_default, "toy_default.png"
            )
        ),
        size_of_map = Dict(
            :toy_default => (width = 10, height = 10)
        ),
        meters_per_pixel = 1.0,
        task_color_for_map = Dict(
            :toy_default => [MAGENTA]
        ),
    )
end


function RobotNavigationPOMDPToyLocalization()
    return RobotNavigationPOMDP(
        maps = Dict(
            :toy_localization => RobotNavigationMap(
                :toy_localization, "toy_localization.png"
            )
        ),
        size_of_map = Dict(
            :toy_localization => (width = 100, height = 100)
        ),
        meters_per_pixel = 0.1,
        task_color_for_map = Dict(
            :toy_localization => [MAGENTA]
        ),
    )
end


function RobotNavigationPOMDPToySLAM()
    return RobotNavigationPOMDP(
        maps = Dict(
            :toy_slam_1 => RobotNavigationMap(
                :toy_slam_1, "toy_slam_1.png"
            ),
            :toy_slam_2 => RobotNavigationMap(
                :toy_slam_2, "toy_slam_2.png"
            ),
            :toy_slam_3 => RobotNavigationMap(
                :toy_slam_3, "toy_slam_3.png"
            ),
            :toy_slam_4 => RobotNavigationMap(
                :toy_slam_4, "toy_slam_4.png"
            ),
        ),
        size_of_map = Dict(
            :toy_slam_1 => (width = 100, height = 100),
            :toy_slam_2 => (width = 100, height = 100),
            :toy_slam_3 => (width = 100, height = 100),
            :toy_slam_4 => (width = 100, height = 100),
        ),
        meters_per_pixel = 0.25,
        task_color_for_map = Dict(
            :toy_slam_1 => [MAGENTA],
            :toy_slam_2 => [MAGENTA],
            :toy_slam_3 => [MAGENTA],
            :toy_slam_4 => [MAGENTA],
        ),
    )
end


function RobotNavigationPOMDPMineSLAM()
    return RobotNavigationPOMDP(
        maps = Dict(
            :mine_slam_1 => RobotNavigationMap(
                :mine_slam_1, "mine_slam_1.png"
            ),
            :mine_slam_2 => RobotNavigationMap(
                :mine_slam_2, "mine_slam_2.png"
            ),
            :mine_slam_3 => RobotNavigationMap(
                :mine_slam_3, "mine_slam_3.png"
            ),
            :mine_slam_4 => RobotNavigationMap(
                :mine_slam_4, "mine_slam_4.png"
            ),
            :mine_slam_5 => RobotNavigationMap(
                :mine_slam_5, "mine_slam_5.png"
            ),
            :mine_slam_6 => RobotNavigationMap(
                :mine_slam_6, "mine_slam_6.png"
            ),
            :mine_slam_7 => RobotNavigationMap(
                :mine_slam_7, "mine_slam_7.png"
            ),
            :mine_slam_8 => RobotNavigationMap(
                :mine_slam_8, "mine_slam_8.png"
            ),
        ),
        size_of_map = Dict(
            :mine_slam_1 => (width = 1280, height = 954),
            :mine_slam_2 => (width = 1280, height = 954),
            :mine_slam_3 => (width = 1280, height = 954),
            :mine_slam_4 => (width = 1280, height = 954),
            :mine_slam_5 => (width = 1280, height = 954),
            :mine_slam_6 => (width = 1280, height = 954),
            :mine_slam_7 => (width = 1280, height = 954),
            :mine_slam_8 => (width = 1280, height = 954),
        ),
        meters_per_pixel = 0.025,
        task_color_for_map = Dict(
            :mine_slam_1 => [MAGENTA],
            :mine_slam_2 => [MAGENTA],
            :mine_slam_3 => [MAGENTA],
            :mine_slam_4 => [MAGENTA],
            :mine_slam_5 => [MAGENTA],
            :mine_slam_6 => [MAGENTA],
            :mine_slam_7 => [MAGENTA],
            :mine_slam_8 => [MAGENTA],
        ),
    )
end

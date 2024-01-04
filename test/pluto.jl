### A Pluto.jl notebook ###
# v0.19.36

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
end

# ╔═╡ 7edb2bb2-a37a-11ee-0609-cf4378ef0dc6
begin
	import Pkg
	Pkg.activate("..")
	
	using Revise
	
	using RobotNavigationPOMDPs
	
	using POMDPs
	using POMDPModels
	using POMDPTools
	using POMDPSimulators
	using ParticleFilters

	using Random
	
	using Pluto
	using PlutoUI
end

# ╔═╡ 7e1244ea-ceb6-4ee5-b38b-f910f0b4d4c7
md"""
---
# Robot Navigation POMDP
Example visualizations and execution code for different `RobotNavigationPOMDP` models is contained within this *Pluto.jl* notebook.

Here is also a helpful button to restart the notebook:
"""

# ╔═╡ 4334bba9-51c4-442b-879e-c02ee215b1a3
HTML("""
<!-- the wrapper span -->
<div>
	<button id="myrestart" href="#">Restart Notebook</button>
	
	<script>
		const div = currentScript.parentElement
		const button = div.querySelector("button#myrestart")
		const cell= div.closest('pluto-cell')
		console.log(button);
		button.onclick = function() { restart_nb() };
		function restart_nb() {
			console.log("Restarting Notebook");
		        cell._internal_pluto_actions.send(                    
		            "restart_process",
                            {},
                            {
                                notebook_id: editor_state.notebook.notebook_id,
                            }
                        )
		};
	</script>
</div>
""")

# ╔═╡ 60a399cf-4dfc-4621-9888-5dab2e7ae0fb
html"""
<style>
	main {
		margin: 0 auto;
		max-width: 2000px;
    	padding-left: max(160px, 10%);
    	padding-right: max(160px, 10%);
	}
</style>
"""

# ╔═╡ a558bb6d-7f3d-4745-b82b-13267aacca77
begin
	N = RobotNavigationAction(desired_move = true, desired_θ = 3.0 * float(π) / 2.0)
	S = RobotNavigationAction(desired_move = true, desired_θ = float(π) / 2.0)
	E = RobotNavigationAction(desired_move = true, desired_θ = 0.0)
	W = RobotNavigationAction(desired_move = true, desired_θ = float(π))
	md"""*Widened width of notebook (above), and defined helper action variables (N, W, E, W).*"""
end

# ╔═╡ a3452c16-90cf-42c1-bc38-fb36c6cf80fc
md"""
---
# Robot Navigation POMDP: Simple Test
"""

# ╔═╡ 23f37d31-3e77-48a5-9ba6-afc6f75b8616
begin
	simple_test_actions = [N, N, N, E, E, E, S, S, W, W, N, N, E, E]
	simple_test_random_steps = 30
	
	simple_test_options = [
		md"""
		1. Random Seed: $(@bind simple_test_seed_slider Slider(1:100, default = 23, show_value = true))
		""",
		md"""
		2. Number of Particles: $(@bind simple_test_num_particles_slider Slider(30:300, default = 30, show_value = true))
		""",
		md"""
		3. Time Step to Visualize: $(@bind simple_test_step_slider Slider(0:(length(simple_test_actions) + simple_test_random_steps), show_value = true))
		""",
	]

	md"""
	**Simple Test Options:**
	$(simple_test_options)
	"""
end

# ╔═╡ 68ed5e1d-ddda-4bcf-9aff-294d9ac9aea6
begin
	simple_test_𝒫 = RobotNavigationPOMDPSimpleTest()
	
	simple_test_rng = MersenneTwister(23)
	
	simple_test_policy = RandomPolicy(simple_test_𝒫, rng = simple_test_rng)
	
	simple_test_random_policy = RandomPolicy(simple_test_𝒫, rng = simple_test_rng)
	simple_test_policy = PlaybackPolicy(simple_test_actions, simple_test_random_policy)
	
	simple_test_max_steps = length(simple_test_actions) + simple_test_random_steps
	simple_test_hr = HistoryRecorder(
		max_steps = simple_test_max_steps, rng = simple_test_rng
	)
	
	simple_test_num_particles = simple_test_num_particles_slider
	simple_test_filter = SIRParticleFilter(
		simple_test_𝒫, simple_test_num_particles, rng = simple_test_rng
	)
	
	simple_test_history = POMDPs.simulate(
		simple_test_hr, simple_test_𝒫, simple_test_policy, simple_test_filter
	)
	
	md"""
	*Simulated random policy with $simple_test_num_particles particles over $simple_test_max_steps steps in total, first $(length(simple_test_actions)) fixed actions then $simple_test_random_steps random actions.*
	"""
end

# ╔═╡ 1ca136a5-3cee-4412-a333-a8205286ce64
begin
	if simple_test_step_slider == 0
		simple_test_step = simple_test_history[1]
		simple_test_𝒱 = robot_navigation_visualizer(simple_test_𝒫, simple_test_step; show_current_state = true)
	else
		simple_test_step = simple_test_history[simple_test_step_slider]
		simple_test_𝒱 = robot_navigation_visualizer(simple_test_𝒫, simple_test_step)
	end
	Base.show(stdout, MIME("text/html"), simple_test_𝒱)
end

# ╔═╡ 86dfcd0d-e681-4494-aac1-4759680bb3bf
md"""
---
# Robot Navigation POMDP: Simple Localization
"""

# ╔═╡ a13f4001-54c0-455a-bfe0-75cb5f03e0cd
begin
	simple_localization_actions = [N, N, N, N, N, N, N, N, N, N, N, N, W, W, W, W, W, S, S, S, S]
	simple_localization_random_steps = 10
	
	simple_localization_options = [
		md"""
		1. Random Seed: $(@bind simple_localization_seed_slider Slider(1:100, default = 23, show_value = true))
		""",
		md"""
		2. Number of Particles: $(@bind simple_localization_num_particles_slider Slider(30:300, default = 30, show_value = true))
		""",
		md"""
		3. Time Step to Visualize: $(@bind simple_localization_step_slider Slider(0:(length(simple_localization_actions) + simple_localization_random_steps), show_value = true))
		""",
	]

	md"""
	**Simple Localization Options:**
	$(simple_localization_options)
	"""
end

# ╔═╡ ba93a383-c6e9-49be-935a-eb83f32445f6
begin
	simple_localization_𝒫 = RobotNavigationPOMDPSimpleLocalization()
	
	simple_localization_rng = MersenneTwister(simple_localization_seed_slider)
	
	simple_localization_random_policy = RandomPolicy(simple_localization_𝒫, rng = simple_localization_rng)
	simple_localization_policy = PlaybackPolicy(simple_localization_actions, simple_localization_random_policy)
	
	simple_localization_max_steps = length(simple_localization_actions) + simple_localization_random_steps
	simple_localization_hr = HistoryRecorder(
		max_steps = simple_localization_max_steps, rng = simple_localization_rng
	)
	
	simple_localization_num_particles = simple_localization_num_particles_slider
	simple_localization_filter = SIRParticleFilter(
		simple_localization_𝒫, simple_localization_num_particles, rng = simple_localization_rng
	)
	
	simple_localization_history = POMDPs.simulate(
		simple_localization_hr, simple_localization_𝒫, simple_localization_policy, simple_localization_filter
	)
	
	md"""
	*Simulated random policy with $simple_localization_num_particles particles over $simple_localization_max_steps steps in total, first $(length(simple_localization_actions)) fixed actions then $simple_localization_random_steps random actions.*
	"""
end

# ╔═╡ 2224a62c-9c29-4cf7-bc31-5cba5ee3ae3e
begin
	if simple_localization_step_slider == 0
		simple_localization_step = simple_localization_history[1]
		simple_localization_𝒱 = robot_navigation_visualizer(simple_localization_𝒫, simple_localization_step; show_current_state = true)
	else
		simple_localization_step = simple_localization_history[simple_localization_step_slider]
		simple_localization_𝒱 = robot_navigation_visualizer(simple_localization_𝒫, simple_localization_step)
	end
	Base.show(stdout, MIME("text/html"), simple_localization_𝒱)
end

# ╔═╡ 5e885808-6d77-433c-bb2f-324154c3b320
md"""
---
# Robot Navigation POMDP: Simple SLAM
"""

# ╔═╡ d016b142-6f05-4d8d-af44-8ee395814204
begin
	simple_slam_actions = [S, S, S, S, S, S, S, S, S, S, S, S, S, S, S, S, S, S, S, S]
	simple_slam_random_steps = 10
	
	simple_slam_options = [
		md"""
		1. Random Seed: $(@bind simple_slam_seed_slider Slider(1:100, default = 23, show_value = true))
		""",
		md"""
		2. Number of Particles: $(@bind simple_slam_num_particles_slider Slider(30:300, default = 30, show_value = true))
		""",
		md"""
		3. Show Most Likely Map: $(@bind simple_slam_show_most_likely_map_checkbox CheckBox(default = true))
		""",
		md"""
		4. Time Step to Visualize: $(@bind simple_slam_step_slider Slider(0:(length(simple_slam_actions) + simple_slam_random_steps), show_value = true))
		""",
	]

	md"""
	**Simple SLAM Options:**
	$(simple_slam_options)
	"""
end

# ╔═╡ 94da4475-3fba-4e7b-91e1-4ccd5998b97f
begin
	simple_slam_𝒫 = RobotNavigationPOMDPSimpleSLAM()
	
	simple_slam_rng = MersenneTwister(simple_slam_seed_slider)
	
	simple_slam_random_policy = RandomPolicy(simple_slam_𝒫, rng = simple_slam_rng)
	simple_slam_policy = PlaybackPolicy(simple_slam_actions, simple_slam_random_policy)
	
	simple_slam_max_steps = length(simple_slam_actions) + simple_slam_random_steps
	simple_slam_hr = HistoryRecorder(
		max_steps = simple_slam_max_steps, rng = simple_slam_rng
	)
	
	simple_slam_num_particles = simple_slam_num_particles_slider
	simple_slam_filter = SIRParticleFilter(
		simple_slam_𝒫, simple_slam_num_particles, rng = simple_slam_rng
	)
	
	simple_slam_history = POMDPs.simulate(
		simple_slam_hr, simple_slam_𝒫, simple_slam_policy, simple_slam_filter
	)
	
	md"""
	*Simulated random policy with $simple_slam_num_particles particles over $simple_slam_max_steps steps in total, first $(length(simple_slam_actions)) fixed actions then $simple_slam_random_steps random actions.*
	"""
end

# ╔═╡ a27520fb-527e-4db1-9699-fbae46072657
begin
	if simple_slam_step_slider == 0
		simple_slam_step = simple_slam_history[1]
		simple_slam_𝒱 = robot_navigation_visualizer(simple_slam_𝒫, simple_slam_step; show_most_likely_map = simple_slam_show_most_likely_map_checkbox, show_current_state = true)
	else
		simple_slam_step = simple_slam_history[simple_slam_step_slider]
		simple_slam_𝒱 = robot_navigation_visualizer(simple_slam_𝒫, simple_slam_step; show_most_likely_map = simple_slam_show_most_likely_map_checkbox)
	end
	Base.show(stdout, MIME("text/html"), simple_slam_𝒱)
end

# ╔═╡ f072424a-4186-41dd-aef8-3405488e920f
md"""
---
# Robot Navigation POMDP: Mine SLAM
"""

# ╔═╡ 30eed6a1-02ef-45bf-b787-39ea37f19f9e
begin
	mine_slam_actions = [E, E, E, N, N, N, N, N, N, N, N, E, E, E, E, E, E, E, E, E]
	mine_slam_random_steps = 10
	
	mine_slam_options = [
		md"""
		1. Random Seed: $(@bind mine_slam_seed_slider Slider(1:100, default = 23, show_value = true))
		""",
		md"""
		2. Number of Particles: $(@bind mine_slam_num_particles_slider Slider(30:300, default = 30, show_value = true))
		""",
		md"""
		3. Show Most Likely Map: $(@bind mine_slam_show_most_likely_map_checkbox CheckBox(default = true))
		""",
		md"""
		4. Time Step to Visualize: $(@bind mine_slam_step_slider Slider(0:(length(mine_slam_actions) + mine_slam_random_steps), show_value = true))
		""",
	]

	md"""
	**Mine SLAM Options:**
	$(mine_slam_options)
	"""
end

# ╔═╡ d6307d63-77f5-4aaf-9256-d6f94e6da7fd
begin
	mine_slam_𝒫 = RobotNavigationPOMDPMineSLAM()
	
	mine_slam_rng = MersenneTwister(mine_slam_seed_slider)
	
	mine_slam_random_policy = RandomPolicy(mine_slam_𝒫, rng = mine_slam_rng)
	mine_slam_policy = PlaybackPolicy(mine_slam_actions, mine_slam_random_policy)
	
	mine_slam_max_steps = length(mine_slam_actions) + mine_slam_random_steps
	mine_slam_hr = HistoryRecorder(
		max_steps = mine_slam_max_steps, rng = mine_slam_rng
	)
	
	mine_slam_num_particles = mine_slam_num_particles_slider
	mine_slam_filter = SIRParticleFilter(
		mine_slam_𝒫, mine_slam_num_particles, rng = mine_slam_rng
	)
	
	mine_slam_history = POMDPs.simulate(
		mine_slam_hr, mine_slam_𝒫, mine_slam_policy, mine_slam_filter
	)
	
	md"""
	*Simulated policy with $mine_slam_num_particles particles over $mine_slam_max_steps steps in total, first $(length(mine_slam_actions)) fixed actions then $mine_slam_random_steps random actions.*
	"""
end

# ╔═╡ 113f11a5-37de-4454-803f-f11fc104c941
begin
	if mine_slam_step_slider == 0
		mine_slam_step = mine_slam_history[1]
		mine_slam_𝒱 = robot_navigation_visualizer(mine_slam_𝒫, mine_slam_step; show_most_likely_map = mine_slam_show_most_likely_map_checkbox, show_current_state = true)
	else
		mine_slam_step = mine_slam_history[mine_slam_step_slider]
		mine_slam_𝒱 = robot_navigation_visualizer(mine_slam_𝒫, mine_slam_step; show_most_likely_map = mine_slam_show_most_likely_map_checkbox)
	end
	Base.show(stdout, MIME("text/html"), mine_slam_𝒱)
end

# ╔═╡ Cell order:
# ╟─7e1244ea-ceb6-4ee5-b38b-f910f0b4d4c7
# ╟─4334bba9-51c4-442b-879e-c02ee215b1a3
# ╟─60a399cf-4dfc-4621-9888-5dab2e7ae0fb
# ╟─a558bb6d-7f3d-4745-b82b-13267aacca77
# ╟─7edb2bb2-a37a-11ee-0609-cf4378ef0dc6
# ╟─a3452c16-90cf-42c1-bc38-fb36c6cf80fc
# ╟─23f37d31-3e77-48a5-9ba6-afc6f75b8616
# ╟─68ed5e1d-ddda-4bcf-9aff-294d9ac9aea6
# ╟─1ca136a5-3cee-4412-a333-a8205286ce64
# ╟─86dfcd0d-e681-4494-aac1-4759680bb3bf
# ╟─a13f4001-54c0-455a-bfe0-75cb5f03e0cd
# ╟─ba93a383-c6e9-49be-935a-eb83f32445f6
# ╟─2224a62c-9c29-4cf7-bc31-5cba5ee3ae3e
# ╟─5e885808-6d77-433c-bb2f-324154c3b320
# ╟─d016b142-6f05-4d8d-af44-8ee395814204
# ╟─94da4475-3fba-4e7b-91e1-4ccd5998b97f
# ╟─a27520fb-527e-4db1-9699-fbae46072657
# ╟─f072424a-4186-41dd-aef8-3405488e920f
# ╟─30eed6a1-02ef-45bf-b787-39ea37f19f9e
# ╟─d6307d63-77f5-4aaf-9256-d6f94e6da7fd
# ╟─113f11a5-37de-4454-803f-f11fc104c941

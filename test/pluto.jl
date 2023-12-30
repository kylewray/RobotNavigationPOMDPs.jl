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
	N = RobotNavigationAction(desired_move = true, desired_θ = float(π) / 2.0)
	S = RobotNavigationAction(desired_move = true, desired_θ = 3.0 * float(π) / 2.0)
	E = RobotNavigationAction(desired_move = true, desired_θ = 0.0)
	W = RobotNavigationAction(desired_move = true, desired_θ = float(π))
	md"""*Widened width of notebook, and defined helper action variables (N, W, E, W).*"""
end

# ╔═╡ a3452c16-90cf-42c1-bc38-fb36c6cf80fc
md"""
---
# Robot Navigation POMDP: Toy Default
"""

# ╔═╡ 68ed5e1d-ddda-4bcf-9aff-294d9ac9aea6
begin
	toy_default_𝒫 = RobotNavigationPOMDPToyDefault()
	
	toy_default_rng = MersenneTwister(23)
	
	toy_default_policy = RandomPolicy(toy_default_𝒫, rng = toy_default_rng)
	
	toy_default_max_steps = 50
	toy_default_hr = HistoryRecorder(
		max_steps = toy_default_max_steps, rng = toy_default_rng
	)
	
	toy_default_num_particles = 300
	toy_default_filter = SIRParticleFilter(
		toy_default_𝒫, toy_default_num_particles, rng = toy_default_rng
	)
	
	toy_default_history = POMDPs.simulate(
		toy_default_hr, toy_default_𝒫, toy_default_policy, toy_default_filter
	)
	
	md"""
	Simulated random policy with $toy_default_num_particles particles over $toy_default_max_steps steps.
	
	Use this slider to visualize any of the steps:
	"""
end

# ╔═╡ d52cdd39-db98-47fd-a5c4-9c5768e850ca
begin
	@bind toy_default_step_slider Slider(0:length(toy_default_history), show_value = true)
end

# ╔═╡ 1ca136a5-3cee-4412-a333-a8205286ce64
begin
	if toy_default_step_slider == 0
		toy_default_step = toy_default_history[1]
		toy_default_𝒱 = robot_navigation_visualizer(toy_default_𝒫, toy_default_step; show_current_state = true)
	else
		toy_default_step = toy_default_history[toy_default_step_slider]
		toy_default_𝒱 = robot_navigation_visualizer(toy_default_𝒫, toy_default_step)
	end
	Base.show(stdout, MIME("text/html"), toy_default_𝒱)
end

# ╔═╡ 86dfcd0d-e681-4494-aac1-4759680bb3bf
md"""
---
# Robot Navigation POMDP: Toy Localization
"""

# ╔═╡ ba93a383-c6e9-49be-935a-eb83f32445f6
begin
	toy_localization_𝒫 = RobotNavigationPOMDPToyLocalization()
	
	toy_localization_rng = MersenneTwister(23)
	
	toy_localization_actions = [N, N, E, E, S, S, W, W]
	toy_localization_random_policy = RandomPolicy(toy_localization_𝒫, rng = toy_localization_rng)
	toy_localization_policy = PlaybackPolicy(toy_localization_actions, toy_localization_random_policy)
	
	toy_localization_max_steps = 10
	toy_localization_hr = HistoryRecorder(
		max_steps = toy_localization_max_steps, rng = toy_localization_rng
	)
	
	toy_localization_num_particles = 500
	toy_localization_filter = SIRParticleFilter(
		toy_localization_𝒫, toy_localization_num_particles, rng = toy_localization_rng
	)
	
	toy_localization_history = POMDPs.simulate(
		toy_localization_hr, toy_localization_𝒫, toy_localization_policy, toy_localization_filter
	)
	
	md"""
	Simulated random policy with $toy_localization_num_particles particles over $toy_localization_max_steps steps.
	
	Use this slider to visualize any of the steps:
	"""
end

# ╔═╡ 1f48b7a9-76d2-46df-94ea-eb396e48046a
@bind toy_localization_step_slider Slider(0:length(toy_localization_history), show_value=true)

# ╔═╡ 2224a62c-9c29-4cf7-bc31-5cba5ee3ae3e
begin
	if toy_localization_step_slider == 0
		toy_localization_step = toy_localization_history[1]
		toy_localization_𝒱 = robot_navigation_visualizer(toy_localization_𝒫, toy_localization_step; show_current_state = true)
	else
		toy_localization_step = toy_localization_history[toy_localization_step_slider]
		toy_localization_𝒱 = robot_navigation_visualizer(toy_localization_𝒫, toy_localization_step)
	end
	Base.show(stdout, MIME("text/html"), toy_localization_𝒱)
end

# ╔═╡ 5e885808-6d77-433c-bb2f-324154c3b320
md"""
---
# Robot Navigation POMDP: Toy SLAM
"""

# ╔═╡ 94da4475-3fba-4e7b-91e1-4ccd5998b97f
begin
	toy_slam_𝒫 = RobotNavigationPOMDPToySLAM()
	
	toy_slam_rng = MersenneTwister(42)
	
	toy_slam_actions = [N, N, N, N, N, N, N, N, N, N, N, N, N, N, N, N, N, N, N, N]
	toy_slam_random_policy = RandomPolicy(toy_slam_𝒫, rng = toy_slam_rng)
	toy_slam_policy = PlaybackPolicy(toy_slam_actions, toy_slam_random_policy)
	
	toy_slam_max_steps = 30
	toy_slam_hr = HistoryRecorder(
		max_steps = toy_slam_max_steps, rng = toy_slam_rng
	)
	
	toy_slam_num_particles = 500
	toy_slam_filter = SIRParticleFilter(
		toy_slam_𝒫, toy_slam_num_particles, rng = toy_slam_rng
	)
	
	toy_slam_history = POMDPs.simulate(
		toy_slam_hr, toy_slam_𝒫, toy_slam_policy, toy_slam_filter
	)
	
	md"""
	Simulated random policy with $toy_slam_num_particles particles over $toy_slam_max_steps steps.
	
	Use this slider to visualize any of the steps:
	"""
end

# ╔═╡ 550f8680-8dac-400d-9049-c1f2c0716b86
@bind toy_slam_step_slider Slider(0:length(toy_slam_history), show_value=true)

# ╔═╡ a27520fb-527e-4db1-9699-fbae46072657
begin
	if toy_slam_step_slider == 0
		toy_slam_step = toy_slam_history[1]
		toy_slam_𝒱 = robot_navigation_visualizer(toy_slam_𝒫, toy_slam_step; show_most_likely_map = true, show_current_state = true)
	else
		toy_slam_step = toy_slam_history[toy_slam_step_slider]
		toy_slam_𝒱 = robot_navigation_visualizer(toy_slam_𝒫, toy_slam_step; show_most_likely_map = true)
	end
	Base.show(stdout, MIME("text/html"), toy_slam_𝒱)
end

# ╔═╡ f072424a-4186-41dd-aef8-3405488e920f
md"""
---
# Robot Navigation POMDP: Mine SLAM
"""

# ╔═╡ d6307d63-77f5-4aaf-9256-d6f94e6da7fd
begin
	mine_slam_𝒫 = RobotNavigationPOMDPMineSLAM()
	
	mine_slam_rng = MersenneTwister(23)
	
	mine_slam_actions = [E, E, E, S, S, S, S, S, S, S, E, E, E, E, E, E, E, S, S, S]
	mine_slam_random_policy = RandomPolicy(mine_slam_𝒫, rng = mine_slam_rng)
	mine_slam_policy = PlaybackPolicy(mine_slam_actions, mine_slam_random_policy)
	
	mine_slam_max_steps = 30
	mine_slam_hr = HistoryRecorder(
		max_steps = mine_slam_max_steps, rng = mine_slam_rng
	)
	
	mine_slam_num_particles = 100
	mine_slam_filter = SIRParticleFilter(
		mine_slam_𝒫, mine_slam_num_particles, rng = mine_slam_rng
	)
	
	mine_slam_history = POMDPs.simulate(
		mine_slam_hr, mine_slam_𝒫, mine_slam_policy, mine_slam_filter
	)
	
	md"""
	Simulated random policy with $mine_slam_num_particles particles over $mine_slam_max_steps steps.
	
	Use this slider to visualize any of the steps:
	"""
end

# ╔═╡ 861c59b4-bbc0-4361-a03b-812ca794d80c
@bind mine_slam_step_slider Slider(0:length(mine_slam_history), show_value=true)

# ╔═╡ 113f11a5-37de-4454-803f-f11fc104c941
begin
	if mine_slam_step_slider == 0
		mine_slam_step = mine_slam_history[1]
		mine_slam_𝒱 = robot_navigation_visualizer(mine_slam_𝒫, mine_slam_step; show_most_likely_map = true, show_current_state = true)
	else
		mine_slam_step = mine_slam_history[mine_slam_step_slider]
		mine_slam_𝒱 = robot_navigation_visualizer(mine_slam_𝒫, mine_slam_step; show_most_likely_map = true)
	end
	Base.show(stdout, MIME("text/html"), mine_slam_𝒱)
end

# ╔═╡ Cell order:
# ╟─7e1244ea-ceb6-4ee5-b38b-f910f0b4d4c7
# ╟─4334bba9-51c4-442b-879e-c02ee215b1a3
# ╟─60a399cf-4dfc-4621-9888-5dab2e7ae0fb
# ╟─a558bb6d-7f3d-4745-b82b-13267aacca77
# ╟─a3452c16-90cf-42c1-bc38-fb36c6cf80fc
# ╟─7edb2bb2-a37a-11ee-0609-cf4378ef0dc6
# ╟─68ed5e1d-ddda-4bcf-9aff-294d9ac9aea6
# ╟─d52cdd39-db98-47fd-a5c4-9c5768e850ca
# ╟─1ca136a5-3cee-4412-a333-a8205286ce64
# ╟─86dfcd0d-e681-4494-aac1-4759680bb3bf
# ╟─ba93a383-c6e9-49be-935a-eb83f32445f6
# ╟─1f48b7a9-76d2-46df-94ea-eb396e48046a
# ╟─2224a62c-9c29-4cf7-bc31-5cba5ee3ae3e
# ╟─5e885808-6d77-433c-bb2f-324154c3b320
# ╟─94da4475-3fba-4e7b-91e1-4ccd5998b97f
# ╟─550f8680-8dac-400d-9049-c1f2c0716b86
# ╟─a27520fb-527e-4db1-9699-fbae46072657
# ╟─f072424a-4186-41dd-aef8-3405488e920f
# ╠═d6307d63-77f5-4aaf-9256-d6f94e6da7fd
# ╟─861c59b4-bbc0-4361-a03b-812ca794d80c
# ╟─113f11a5-37de-4454-803f-f11fc104c941

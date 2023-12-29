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

# ╔═╡ 68ed5e1d-ddda-4bcf-9aff-294d9ac9aea6
begin
	𝒫 = RobotNavigationPOMDP()
	policy = RandomPolicy(𝒫)
	
	rng = MersenneTwister(23)
	
	hr = HistoryRecorder(max_steps = 50, rng = rng)
	filter = SIRParticleFilter(𝒫, 300, rng = rng)
	history = POMDPs.simulate(hr, 𝒫, policy, filter)
end

# ╔═╡ d52cdd39-db98-47fd-a5c4-9c5768e850ca
@bind stepSlider Slider(1:length(history))

# ╔═╡ 1ca136a5-3cee-4412-a333-a8205286ce64
begin
	step = history[stepSlider]
	
	𝒱 = robot_navigation_visualizer(𝒫, step)

	Base.show(stdout, MIME("text/html"), 𝒱)
end

# ╔═╡ Cell order:
# ╟─4334bba9-51c4-442b-879e-c02ee215b1a3
# ╠═7edb2bb2-a37a-11ee-0609-cf4378ef0dc6
# ╠═68ed5e1d-ddda-4bcf-9aff-294d9ac9aea6
# ╠═d52cdd39-db98-47fd-a5c4-9c5768e850ca
# ╠═1ca136a5-3cee-4412-a333-a8205286ce64

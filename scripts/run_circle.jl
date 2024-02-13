using DrWatson
@quickactivate "Essaim"

include(srcdir("RobotCircle.jl"))
include(srcdir("Draw.jl"))

model = Circle.initialize_model(; seed = 1, δt = 0.001, N = 15, com_range = 25., vis_range = 25., history_size = 500, extent=(100.,100.), speed=1.0)

# fig, plot_dict, interaction_dict = make_figure(model, agent_laplacian_step!)

#run_animation!(model, agent_laplacian_step!; n_steps=2500)

#make_animation!(model, agent_simple_step!; n_frames=1000, steps_per_frame=1)

Draw.run_simulator!(model, Circle.agent_laplacian_step!)

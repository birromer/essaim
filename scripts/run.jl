using DrWatson
@quickactivate "Essaim"

include(srcdir("Robot.jl"))
include(srcdir("Draw.jl"))

model = initialize_model(; seed = 1, δt = 0.001, N = 78, com_range = 25., vis_range = 50., history_size = 500, extent=(300.,300.), speed=1.0)

# fig, plot_dict, interaction_dict = make_figure(model, agent_laplacian_step!)

run_simulator!(model, agent_laplacian_step!)

#run_animation!(model, agent_laplacian_step!; n_steps=2500)

#run_animation!(model, agent_simple_step!; n_steps=1000)

#make_animation!(model, agent_simple_step!; n_frames=1000, steps_per_frame=1)


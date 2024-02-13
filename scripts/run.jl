using DrWatson
@quickactivate "Essaim"

include(srcdir("Robot.jl"))
include(srcdir("RobotCircle.jl"))
include(srcdir("Draw.jl"))

model_d = Dubins.initialize_model(; seed = 1, δt = 0.001, N = 15, com_range = 25., vis_range = 25., history_size = 500, extent=(100.,100.), speed=1.0)
model_c = Circle.initialize_model(; seed = 1, δt = 0.001, N = 15, com_range = 25., vis_range = 25., history_size = 500, extent=(100.,100.), speed=1.0)

# fig, plot_dict, interaction_dict = make_figure(model, agent_laplacian_step!)

#Draw.run_animation!(model_d, Dubins.agent_laplacian_step!; n_steps=2500)
Draw.run_animation!(model_c, Circle.agent_laplacian_step!; n_steps=2500)

#run_animation!(model, agent_simple_step!; n_steps=1000)

#make_animation!(model, agent_simple_step!; n_frames=1000, steps_per_frame=1)

#Draw.run_simulator!(model, agent_laplacian_step!)

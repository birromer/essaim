using DrWatson
@quickactivate "Essaim"

include(srcdir("Dubins/Robot.jl"))
include(srcdir("Dubins/Draw.jl"))

model = Robot.initialize_model(; seed = 1, δt = 0.001, N = 15, com_range = 25., vis_range = 25., history_size = 500, extent=(100.,100.), speed=1.0)

#Draw.run_animation!(m_dubins, Robot.agent_laplacian_step!; n_steps=2500)

#fig, plot_dict, interaction_dict = make_figure(m_dubins, RobotOntic.agent_laplacian_step!)

#Draw.make_animation!(m_dubins, Robot.agent_simple_step!; n_frames=1000, steps_per_frame=1)

Draw.run_simulator!(model, Robot.agent_laplacian_step!, Robot.initialize_model)

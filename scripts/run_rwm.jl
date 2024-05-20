using DrWatson
@quickactivate "Essaim"

include(srcdir("with_memory", "RobotWithMemory.jl"))

model = RobotWithMemory.initialize_model(;
                               N = 10,
                               dimensions = (2, 1, 0, 2),
                               range_dims = ((75.0, 75.0),
                                             (75.0, 75.0),
                                             (0, 2π)),
                               copy_ontic = 2,
                               extent = (100.,100.),
                               com_range = 25.,
                               vis_range = 25.,
                               history_size = 500,
                               speed = 1.0,
                               δt = 0.001,
                               seed = 1,
                              )

#Draw.run_animation!(m_dubins, Robot.agent_laplacian_step!; n_steps=2500)

#Draw.make_animation!(m_dubins, Robot.agent_simple_step!; n_frames=1000, steps_per_frame=1)

RobotWithMemory.run_simulator!(model, RobotWithMemory.agent_step!, RobotWithMemory.initialize_model)

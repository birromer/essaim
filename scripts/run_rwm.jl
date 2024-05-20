using DrWatson
@quickactivate "Essaim"

include(srcdir("with_memory", "RobotWithMemory.jl"))

model = RobotWithMemory.initialize_model(
                         RobotWithMemory.agent_laplacian_step!;
                         N = 10,                      # number of agents
                         dimensions = (2,             # dim ontic position
                                       1,             # dim ontic rotation
                                       0,             # dim other ontic
                                       2),            # dim epi state
                         range_dims = ((75.0, 75.0),  # range pos x
                                       (75.0, 75.0),  # range pos y
                                       (0, 2π)),      # range rotations
#                                      (2, 15),       # range of each other
#                                      (-3, 3),       #  ontic state variable
                         copy_ontic = 2,          # initialize first n values
                                                  #  of epistemic from ontic
                         extent = (100.0,100.0),  # size of the world
                         speed = 1.0,             # their initial speed
                         vis_range = 100.0,       # visibility range
                         com_range = 100.0,       # communication range
                         δt = 0.001,               # time step of the model
                         history_size = 500,      # N saved past states
                         seed = 42                # random seed
                        )

#Draw.run_animation!(m_dubins, Robot.agent_laplacian_step!; n_steps=2500)

#Draw.make_animation!(m_dubins, Robot.agent_simple_step!; n_frames=1000, steps_per_frame=1)

RobotWithMemory.run_simulator!(model, RobotWithMemory.initialize_model)

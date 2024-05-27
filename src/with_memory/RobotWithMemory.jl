module RobotWithMemory
export Robot,
       initialize_model,
       run_animation!,
       make_animation!,
       run_simulator!,
       agent_laplacian_step!,
       agent_flock_step!,
       agent_test_step!

include("model.jl")
include("robot.jl")
#TODO: Implement other dynamics for robot.
include("draw.jl")  # Only for 2D visualization
#TODO: Fix graph visualization.

# example usage

#model = initialize_model(agent_step!)

#run_animation!(model, agent_step!; n_steps=2500)

#run_animation!(model, agent_step!; n_steps=1000)

#make_animation!(model, agent_step!; n_frames=1000, steps_per_frame=1)

#run_simulator!(model, initialize_model)

end # module

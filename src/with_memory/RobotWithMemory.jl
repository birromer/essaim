module RobotWithMemory
export Robot,
       initialize_model,
       agent_step!,
       agent_test_step!,
       run_animation!,
       make_animation!,
       run_simulator!

include("model.jl")
include("robot.jl")
include("draw.jl")  # Only for 2D visualization

# example usage

#model = initialize_model()

#run_animation!(model, agent_step!; n_steps=2500)

#run_animation!(model, agent_step!; n_steps=1000)

#make_animation!(model, agent_step!; n_frames=1000, steps_per_frame=1)

run_simulator!(model, agent_step!, initialize_model)

end # module

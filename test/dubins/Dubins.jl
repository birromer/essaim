module Dubins
export Robot,
       initialize_model,
       agent_step!,
       agent_test_step!,
       run_animation!,
       make_animation!,
       run_simulator!

include("model.jl")
include("draw.jl")
include("robot.jl")

# example usage

#model = initialize_model(; seed = 1, δt = 0.001, N = 10, com_range = 25., vis_range = 50., history_size = 300, extent=(100.,100.), speed=1.0)

#run_animation!(model, agent_laplacian_step!; n_steps=2500)

#run_animation!(model, agent_simple_step!; n_steps=1000)

#make_animation!(model, agent_simple_step!; n_frames=1000, steps_per_frame=1)

#run_simulator!(model, agent_simple_step!, initialize_model)

end # module

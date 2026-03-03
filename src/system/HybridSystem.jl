module HybridSystem
export Robot,
       initialize_model,
       run_animation!,
       make_animation!,
       run_simulator!,
       agent_generic_step!,
       agent_flock_step!,
       agent_test_step!

include("model.jl")
include("robot.jl")
include("draw.jl")  # Only for 2D visualization on continuous space

end # module

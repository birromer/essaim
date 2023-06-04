module Essaim
export Robot, initialize_model, agent_laplacian_step!, run_simulation!, make_animation!, run_animation!

using DrWatson
@quickactivate "Essaim"

include(srcdir("Draw.jl"))

function julia_main()::Cint
    # do something based on ARGS?
    model = initialize_model(; seed = 1, δt = 0.001, N = 15, com_range = 25., vis_range = 25., history_size = 500, extent=(100.,100.), speed=1.0)


    run_simulator!(model, agent_laplacian_step!)

    return 0 # if things finished successfully
end

end

using DrWatson
@quickactivate "Essaim"

include(srcdir("Robot.jl"))
include(srcdir("Draw.jl"))


model = initialize_model(;δt=0.001, history_size=500)

fig, rob_pos, rob_hist, rob_marker, rob_color = make_figure(model)

for i ∈ 1:500
    animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_color)
    sleep(model.δt)
end

frames = 1:1000
record(fig, plotsdir("essaim.mp4"), frames; framerate = 60) do i
    for j in 1:3  # each frame is stepped 3 times
        animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_color)
    end
end

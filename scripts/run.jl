using DrWatson
@quickactivate "Essaim"

include(srcdir("Robot.jl"))
include(srcdir("Draw.jl"))


model = initialize_model(;N=40, δt=0.001, history_size=500, com_range=25., vis_range=25.)

fig, rob_pos, rob_hist, rob_color, rob_rot = make_figure(model)

frames = 1:3000

for i ∈ frames
    animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_color, rob_rot)
    sleep(model.δt)
end

record(fig, plotsdir("essaim.mp4"), frames; framerate = 60) do i
    for j in 1:3  # each frame is stepped 3 times
        animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_color, rob_rot)
    end
end

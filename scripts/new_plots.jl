using DrWatson
@quickactivate "Essaim"

include(srcdir("Robot.jl"))

using Agents
using GLMakie
using GraphMakie
using Graphs
using InteractiveDynamics
using DataStructures: CircularBuffer
using Random
using Makie.Colors
using Printf

# base robot, center (0,0), facing east (0rad), with length 1 and base 0.5
const robot_poly = Polygon(Point2f[(0.5, 0.), (-0.5, 0.25), (-0.5,-0.25), (0.5, 0.)])

# helper functions for plotting
robot_status(robot::Robot) = robot.alive == true ? :darkgreen : :red

function visibility_graph(model)
    neighbours = [nearby_ids(r, model, r.vis_range) for r in allagents(model)]

    g = SimpleGraph(nagents(model))
    for (id1,r1) in enumerate(allagents(model))
        for id2 in nearby_ids(r1, model, r1.vis_range)
            if id1 != id2
                add_edge!(g, id1, id2)
            end
        end
    end

    g
end

#robot_marker(robot::Robot) = rotate2D(robot_poly, robot.θ)
#rob_marker = Observable(robot_marker.(allagents(model)))
#r1 = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
model = initialize_model(;N=20, δt=0.001, history_size=500)

# initialize figure for the animation
function make_figure(model)
    N = nagents(model)

    # first we generate the markers for the current state of the robots
    rob_pos = Observable([Point2f(r.pos[1], r.pos[2]) for r in allagents(model)])

    # generate the tail with all potions in the same place initially
    rob_hist = [Observable(CircularBuffer{Point2f}(model.history_size)) for _ in 1:N]
    for (id, buf) in enumerate(rob_hist)
        fill!(buf[], rob_pos[][id])
    end

    # make the properties that change according to the robot observables
    rob_status = Observable(robot_status.(allagents(model)))  # the countour color
    rob_rot = Observable([r.θ for r in allagents(model)])

    # make the togglable properties of the plot observables too
    rob_vis = Observable([r.vis_range for r in allagents(model)])
    rob_com = Observable([r.com_range for r in allagents(model)])

    # different color for each robot
    traj_color = [[RGBAf(model.colors[r].r, model.colors[r].g, model.colors[r].b, (i/model.history_size)^2) for i in 1:model.history_size] for r in 1:N]

    # make the visibility graph to be displayed in the side
    vis_graph = Observable(visibility_graph(model))

    rob_plot = Dict(
        "pos" => rob_pos,
        "hist" => rob_hist,
        "status" => rob_status,
        "rot" => rob_rot,
        "vis" => rob_vis,
        "com" => rob_com,
        "graph" => vis_graph
    )

    # create a new figure
    #fig = Figure(;resolution=(1200,800));
    fig = Figure(); display(fig)

    ax_main  = Axis(
        fig[1:16,1:4],
        title="Dynamic continuous average consensus",
        aspect = DataAspect(),
        limits = (0,model.space.extent[1], 0, model.space.extent[2]),
        xminorticksvisible = true,
        xminorticks = IntervalsBetween(5),
        yminorticksvisible = true,
        yminorticks = IntervalsBetween(5)
    )

    ax_graph = Axis(
        fig[7:15,5:6],
        title="Visibility graph",
        aspect = DataAspect(),
    )
    hidedecorations!(ax_graph)

    fig[6, 5:6] = buttongrid = GridLayout(tellwidth = false)
    buttonlabels = ["Run/Stop", "Step", "Reset"]
    buttons = buttongrid[1, 1:3] = [Button(fig, label = l) for l in buttonlabels]

    args_label = (;justification = :left, halign = :left)
    Label(fig[2, 5], "Number of agents:"; args_label...)
    Label(fig[3, 5], "Time step:"; args_label...)
    Label(fig[4, 5], "Visibility radius:"; args_label...)
    Label(fig[5, 5], "Seed:"; args_label...)

    args_tb = (; halign = :left, textpadding = (6,6,6,6), validator = Float64)
    tb_agents     = Textbox(fig[2, 6], placeholder = "25 agents"; args_tb...)
    tb_time       = Textbox(fig[3, 6], placeholder = "0.01 seconds"; args_tb...)
    tb_visibility = Textbox(fig[4, 6], placeholder = "25 meters"; args_tb...)
    tb_seed       = Textbox(fig[5, 6], placeholder = "42"; args_tb...)

    on(tb_agents.stored_string) do s
        frequency[] = parse(Float64, s)
    end

    scatter!(ax_main, rob_plot["pos"];
             marker = robot_poly,
             rotations = rob_plot["rot"],
             strokewidth = 3,
             strokecolor = rob_plot["status"], # contour color is status (alive/dead)
             color = model.colors,     # unique color, same for trail
             markersize = 11
    )

    # plot trajectories
    for id in allids(model)
        lines!(ax_main, rob_plot["hist"][id];
               linewidth = 3,
               color = traj_color[id]
        )
    end

    # plot visibility range
    for id in allids(model)
        arc!(ax_main, model[id].pos, model[id].vis_range, -π, π;
               linestyle = :dash,
               linewidth = 1,
               color = model.colors[id]
        )
    end

    # generate visibility graph with id colors
    graphplot!(ax_graph, rob_plot["graph"], node_color=model.colors)

    return fig, rob_plot
end

# this function updates the data in the model using the robot step then updates
# the observables (positions, trajectory and markers) related to the plot
#function animation_step!(model, agent_step!, rob_pos, rob_hist, rob_status, rob_rot, vis_graph)
function animation_step!(model, agent_step!, rob_plot)
    # update robot positions
    step!(model, agent_step!)

    # update positions
    rob_plot["pos"][] = [Point2f(r.pos[1], r.pos[2]) for r in allagents(model)]

    # update trajectories
    for (id, buf) in enumerate(rob_plot["hist"])
        push!(buf[], rob_plot["pos"][][id])
        buf[] = buf[]
    end

    # update the markers orientation and color
    rob_plot["status"][] = robot_status.(allagents(model))
    rob_plot["rot"][] = [r.θ for r in allagents(model)]

    # update visibility graph
    rob_plot["graph"][] = visibility_graph(model)
    return
end

# example usage
#
# r1 = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
# robot_marker(r1)
# robot_color(r1)
# model = initialize_model(;extent=(10.,10.))
# agent_simple_step!(r1, model)

model = initialize_model(;
                         history_size=500,
                         δt=0.01,
                         N=5,
                         com_range=10.,
                         vis_range=10.,
)

fig, rob_plot = make_figure(model)

for i ∈ 1:2500
    animation_step!(model, agent_simple_step!, rob_plot)
    sleep(model.δt)
end
#
# frames = 1:1000
# record(fig, plotsdir("essaim.mp4"), frames; framerate = 60) do i
#     for j in 1:3  # each frame is stepped 3 times
#         animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_status)
#     end
# end

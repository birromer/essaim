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
using Colors

# base robot, center (0,0), facing east (0rad), with length 1 and base 0.5
const robot_poly = Polygon(Point2f[(0.5, 0.), (-0.5, 0.25), (-0.5,-0.25), (0.5, 0.)])

# helper functions for plotting
robot_status(robot::Robot) = robot.alive == true ? :darkgreen : :red

function visibility_graph(model)
    N = nagents(model)
    neighbours = [nearby_ids(r, model, r.vis_range) for r in allagents(model)]

    m = [[i in neighbours[r] ? 1 : 0 for i in 1:N] for r in 1:N]
end

#robot_marker(robot::Robot) = rotate2D(robot_poly, robot.θ)
#rob_marker = Observable(robot_marker.(allagents(model)))
#r1 = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
model = initialize_model(;N=20, δt=0.001, history_size=500)

# initialize figure for the animation
function make_figure(model)
    N = nagents(model)
    # first we generate the markers for the current state of the robots
    #rob_pos = Observable([Point3f(r.pos[1], r.pos[2], r.θ) for r in allagents(model)])
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

    # create a new figure
    fig = Figure(;resolution=(1200,800)); display(fig)
    ax_main  = Axis(fig[1:4,1], title="Dynamic continuous average consensus")
    ax_graph = Axis(fig[4:3,2], title="Visibility graph")

    ax_main.aspect = DataAspect()
    ax_graph.aspect = DataAspect()
    xlims!(ax_main, 0, model.space.extent[1])
    ylims!(ax_main, 0, model.space.extent[2])

    fig[2, 2] = buttongrid = GridLayout(tellwidth = false)
    buttonlabels = ["Start", "Stop", "Reset"]
    buttons = buttongrid[1, 1:3] = [Button(fig, label = l) for l in buttonlabels]

    sg = SliderGrid(
        fig[1, 2],
        (label = "Time step", range = 0:0.01:0.5, format = "{:.1f}s", startvalue = model.δt),
        (label = "Visibility range", range = 0:0.1:model.space.extent[1], format = "{:.1f}m", startvalue = 10.),
        (label = "Communication range", range = 0:0.1:model.space.extent[1], format = "{:.1f}m", startvalue = 10.),
        width = 350,
        tellheight = false
    )

    # plot the robots with triangles
    scatter!(ax_main, rob_pos;
             marker = robot_poly,
             rotations = rob_rot,
             strokewidth = 3,
             strokecolor = rob_status, # contour color is status (alive/dead)
             color = model.colors,     # unique color, same for trail
             markersize = 11
    )

    # plot the trajectory of each of them
    # TODO update this for the case new ones appear

    # plot trajectories
    for id in allids(model)
        lines!(ax_main, rob_hist[id];
               linewidth = 3,
               color = traj_color[id]
        )
    end

    # plot visibility range
#    for id in allids(model)
#        arc!(ax_main, rob_pos, rob_vis;
#               linestyle = :dash,
#               linewidth = 2,
#               color = traj_color[id]
#        )
#    end
#
#    # plot communication range
#    for id in allids(model)
#        arc!(ax_main, rob_pos, rob_com;
#               linestyle = :dot,
#               linewidth = 2,
#               color = traj_color
#        )
#    end

    # generate visibility graph with id colors
#    g = visibility_graph(model)
#    graphplot!(ax_graph, g)

    return fig, rob_pos, rob_hist, rob_status, rob_rot
end

# this function updates the data in the model using the robot step then updates
# the observables (positions, trajectory and markers) related to the plot
function animation_step!(model, agent_step!, rob_pos, rob_hist, rob_status, rob_rot)
    # update robot positions
    step!(model, agent_step!)

    # update positions
    rob_pos[] = [Point2f(r.pos[1], r.pos[2]) for r in allagents(model)]

    # update trajectories
    for (id, buf) in enumerate(rob_hist)
        push!(buf[], rob_pos[][id])
        buf[] = buf[]
    end

    # update the markers orientation and color
    rob_status[] = robot_status.(allagents(model))
    rob_rot[] = [r.θ for r in allagents(model)]
    return
end

# example usage
#
# r1 = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
# robot_marker(r1)
# robot_color(r1)
# model = initialize_model(;extent=(10.,10.))
# agent_simple_step!(r1, model)
#
model = initialize_model(;δt=0.001, history_size=500, N=10, com_range=25., vis_range=25.)

fig, rob_pos, rob_hist, rob_status, rob_rot = make_figure(model)

for i ∈ 1:2500
    animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_status, rob_rot)
    sleep(model.δt)
end
#
# frames = 1:1000
# record(fig, plotsdir("essaim.mp4"), frames; framerate = 60) do i
#     for j in 1:3  # each frame is stepped 3 times
#         animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_status)
#     end
# end

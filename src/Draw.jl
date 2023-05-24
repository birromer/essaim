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

# base robot, center (0,0), facing east (0rad), with length 1 and base 0.5
const robot_poly = Polygon(Point2f[(0.5, 0.), (-0.5, 0.25), (-0.5,-0.25), (0.5, 0.)])

# helper functions for plotting
robot_color(robot::Robot) = robot.alive == true ? :darkgreen : :red

#robot_marker(robot::Robot) = rotate2D(robot_poly, robot.θ)
#rob_marker = Observable(robot_marker.(allagents(model)))
#r1 = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
#model = initialize_model(;N=40, δt=0.001, history_size=500)

# initialize figure for the animation
function make_figure(model)
    # first we generate the markers for the current state of the robots
    #rob_pos = Observable([Point3f(r.pos[1], r.pos[2], r.θ) for r in allagents(model)])
    rob_pos = Observable([Point2f(r.pos[1], r.pos[2]) for r in allagents(model)])

    # generate the tail with all potions in the same place initially
    rob_hist = [Observable(CircularBuffer{Point2f}(model.history_size)) for _ in 1:nagents(model)]
    for (id, buf) in enumerate(rob_hist)
        fill!(buf[], rob_pos[][id])
    end

    # make the markers also observables, so that we can update the orientation too
    rob_color = Observable(robot_color.(allagents(model)))
    rob_rot = Observable([r.θ for r in allagents(model)])

    # create a new figure
    fig = Figure(;resolution=(900,800)); display(fig)
    ax = Axis(fig[1,1])

    ax.title = "Dynamic continuous average consensus"
    ax.aspect = DataAspect()
    xlims!(ax, 0, model.space.extent[1])
    ylims!(ax, 0, model.space.extent[2])

    # plot the robots with triangles
    scatter!(ax, rob_pos;
             marker = robot_poly,
             rotations = rob_rot,
             strokewidth = 2,
             strokecolor = rob_color,
             color = :black,
             markersize = 11
    )

    # plot the trajectory of each of them
    c = to_color(:orange)
    traj_color = [RGBAf(c.r, c.g, c.b, (i/model.history_size)^3) for i in 1:model.history_size]

    for id in allids(model)
        lines!(ax, rob_hist[id];
               linewidth = 3,
               color = traj_color
        )
    end

    return fig, rob_pos, rob_hist, rob_color, rob_rot
end

# this function updates the data in the model using the robot step then updates
# the observables (positions, trajectory and markers) related to the plot
function animation_step!(model, agent_step!, rob_pos, rob_hist, rob_color, rob_rot)
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
    rob_color[] = robot_color.(allagents(model))
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
# model = initialize_model(;δt=0.001, history_size=500)
#
# fig, rob_pos, rob_hist, rob_marker, rob_color = make_figure(model)
#
# for i ∈ 1:500
#     animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_color)
#     sleep(model.δt)
# end
#
# frames = 1:1000
# record(fig, plotsdir("essaim.mp4"), frames; framerate = 60) do i
#     for j in 1:3  # each frame is stepped 3 times
#         animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_color)
#     end
# end

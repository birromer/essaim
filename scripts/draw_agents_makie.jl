using DrWatson
@quickactivate "Essaim"

using Agents
using GLMakie
using InteractiveDynamics
using DataStructures: CircularBuffer
using Random


# our robots are in the dubin's car model
# cannot subtype ContinuousAgent as it's concrete
mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Float64} # pos and vel required like this for ContinuousAgent
    vel::NTuple{D,Float64} # required for using update_vel! in the model
    θ::Float64
    θ̇::Float64
    vis_range::Float64  # how far can it see?
    com_range::Float64  # how far can it communicate?
    alive::Bool  # is it alive?
end

# here we initialize the model with its parameters and populate with robots
function initialize_model(;
                          N = 5,                 # number of agents
                          extent = (20.0,20.0),  # size of the world
                          speed = 1.0,           # their initial velocity
                          vis_range = 5.0,       # visibility range
                          com_range = 5.0,       # communication range
                          δt = 0.05,             # time step of the model
                          history_size = 150,     # amount of saved past states
                          seed = 42              # random seed
)
    # initialize model
    space = ContinuousSpace(extent)  # 2D euclidean space
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                                    # in order of their indexing
    D = length(extent)  # number of dimensions
    properties = Dict(  # save the time step in the model
        :δt => δt,
        :history_size => history_size
    )

    model = AgentBasedModel(Robot{D}, space;
                rng = rng,
                scheduler = scheduler,
                properties = properties
    )

    # now we add the agents
    for n ∈ 1:N
        # get random position and heading
        pos = Tuple(rand(model.rng, 2) .* extent)
        heading = rand(model.rng) * 2π
        vel = (speed, speed)

        # initialize the agents with argument values and no heading change
        agent = Robot{D}(n, pos, vel, heading, 0.0, vis_range, com_range, true)
        add_agent!(agent, model)
    end

    return model
end

# we define how each robot behaves, at
function agent_laplacian_step!(robot, model)

end

# a sim behaviour for the robots
# - they keep a low speed with 0 rotation speed
# - unless they reach further than X meters from the nearest neighbor,
# then they rotate
function agent_simple_step!(robot, model)
    neighbor_ids = nearby_ids(robot, model, robot.vis_range)

    robot.pos += robot.vel * model.δt
    robot.θ += robot.θ̇ * model.δt

    robot.vel = (
        robot.pos[1]*cos(robot.θ) - robot.pos[2]*sin(robot.θ),
        robot.pos[1]*sin(robot.θ) + robot.pos[2]*cos(robot.θ)
    )

    for id in neighbor_ids
        if euclidean_distance(robot.pos, model[id].pos, model) > robot.com_range
            robot.θ̇ += 0.05
            break
        end
    end

    return
end

# base robot, center (0,0), facing east (0rad), with length 1 and base 0.5
const robot_poly = Polygon(Point2f[(0.5, 0.) ,(-0.5, 0.25) ,(-0.5,-0.25) ,( 0.5, 0.)])

# helper functions for plotting
robot_marker(robot::Robot) = rotate2D(robot_poly, r.θ)
robot_color(robot::Robot) = robot.alive == true ? :darkgreen : :red

# test
r = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
robot_marker(r)
robot_color(r)

model = initialize_model(;extent=(10.,10.))

# initialize figure for the animation
function make_figure(model)

    # first we generate the markers for the current state of the robots
    #rob_pos = Observable([Point3f(r.pos[1], r.pos[2], r.θ) for r in allagents(model)])
    rob_pos = Observable([Point2f(r.pos[1], r.pos[2]) for r in allagents(model)])

    # generate the tail with all potions in the same place initially
    rob_hist = [Observable(CircularBuffer{Point2f}(model.history_size)) for _ in 1:nagents(model)]
    for id in allids(model)
        fill!(rob_hist[id][], rob_pos[][id])
    end

    # make the markers also observables, so that we can update the orientation too
    rob_marker = Observable(robot_marker.(allagents(model)))
    rob_color = Observable(robot_color.(allagents(model)))

    # create a new figure
    fig = Figure(); display(fig)
    ax = Axis(fig[1,1])

    ax.title = "Dynamic continuous average consensus"
    ax.aspect = DataAspect()
    xlims!(ax, 0, model.space.extent[1])
    ylims!(ax, 0, model.space.extent[2])

    # plot the robots with triangles
    scatter!(ax, rob_pos;
             marker = rob_marker,
             strokewidth = 2,
             strokecolor = rob_color,
             color = :black,
             markersize = 10
    )

    # plot the trajectory of each of them
    c = to_color(:orange)
    traj_color = [RGBAf(c.r, c.g, c.b, (i/model.history_size)^2) for i in 1:model.history_size]

    for id in allids(model)
        lines!(ax, rob_hist[id];
               linewidth = 3,
               color = traj_color
        )
    end

    return fig, rob_pos, rob_hist, rob_marker, rob_color
end

# this function updates the data in the model using the robot step then updates
# the observables (positions, trajectory and markers) related to the plot
function animation_step!(model, agent_step!, rob_pos, rob_hist, rob_marker, rob_color)
    # update robot positions
    step!(model, agent_step!)

    # update positions
    rob_pos[] = [Point2f(r.pos[1], r.pos[2]) for r in allagents(model)]

    # update trajectories
    for (id, buf) in enumerate(rob_hist)
        push!(buf[], rob_pos[][id])
    end

    # update the markers orientation and color
    rob_marker[] = robot_marker.(allagents(model))
    rob_color[] = robot_color.(allagents(model))
end

model = initialize_model(;extent=(10.,10.))

for robot in allagents(model)
    agent_simple_step!(robot, model)
end

robot_1 = model.agents[1]
neighbor_ids = nearby_ids(robot_1, model, robot_1.vis_range)

robot_1.pos += robot_1.vel .* model.δt
robot_1.θ += robot_1.θ̇ .* model.δt

robot_1.vel = (
    robot_1.pos[1]*cos(robot_1.θ) - robot_1.pos[2]*sin(robot_1.θ),
    robot_1.pos[1]*sin(robot_1.θ) + robot_1.pos[2]*cos(robot_1.θ)
)

for id in neighbor_ids
    if euclidean_distance(robot_1.pos, model[id].pos, model) > robot_1.com_range
        robot_1.θ̇ += 0.05
        break
    end
end

fig, rob_pos, rob_hist, rob_marker, rob_color = make_figure(model)

animation_step!(model, agent_simple_step!, rob_pos, rob_hist, rob_marker, rob_color)

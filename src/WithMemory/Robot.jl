module RobotWithMemory 
export Robot, initialize_model, agent_laplacian_step!, agent_simple_step!

using DrWatson
@quickactivate "Essaim"

using Agents
using DataStructures: CircularBuffer
using Random
using Colors

# mutable struct OnticStateDubins2D
#     pos::NTuple{2,Float64}
#     vel::NTuple{2,Float64}
#     θ::Float64
#     dθ::Float64
# end
# 
# # For a 2d dubins, D=3 (pos[2] + rot[1], vel + pre)
# # For a 3d dubins, D=6 (pos[3] + rot[3], vel + pre)
# mutable struct OnticState{D}
#     x::NTuple{D,Float64}
#     dx::NTuple{D,Float64}
# end
# 
# mutable struct EpistemicState{D}
#     μ::NTuple{D,Float64}
#     ̇dμ::NTuple{D,Float64}
# end
# mutable struct Robot{D, OnticState, EpistemicState} <: AbstractAgent
#     id::Int
#     onc::OnticState{D}      # The ontic and epistemic  states are provided
#     epi::EpistemicState{D}  # according to the task implemented
# 
#     vis_range::Float64  # Perception range
#     com_range::Float64  # Communication range
#     alive::Bool  # Is it alive?
# end

#TODO: What is there inside of AbstractAgent?
mutable struct Robot{O,E} <: AbstractAgent
    id::Int

    x::NTuple{O,Float64}   # ontic state
    dx::NTuple{O,Float64}  # d/dt ontic state

    μ::NTuple{E,Float64}   # epistemic state
    ̇dμ::NTuple{E,Float64}  # d/dt epistemic state

    vis_range::Float64  # Perception range
    com_range::Float64  # Communication range
    alive::Bool  # Is it alive?
end

# here we initialize the model with its parameters and populate with robots
function initialize_model(;
                          dimensions = (2,             # dim ontic position
                                        1,             # dim ontic rotation
                                        0,             # dim other ontic
                                        2),            # dim epi state
                          range_dims = ((75.0, 75.0),  # range pos x
                                        (75.0, 75.0),  # range pos y
                                        (0, 2π)),      # range rotations
#                                       (2, 15),       # range of each other
#                                       (-3, 3),       #  ontic state variable
                          copy_ontic = 2,          # initialize first n values
                                                   #  of epistemic from ontic
                          N = 10,                  # number of agents
                          extent = (100.0,100.0),  # size of the world
                          speed = 1.0,             # their initial speed
                          vis_range = 100.0,       # visibility range
                          com_range = 100.0,       # communication range
                          δt = 0.01,               # time step of the model
                          history_size = 300,      # N saved past states
                          seed = 42                # random seed
)
    # initialize model
    space = ContinuousSpace(extent)     # N-D euclidean space
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest      # they are executed semi-synchronously,
                                        #  in order of their indexing
    # get dimensions
    D_onc_pos = dimensions[1]    # position dimensions
    D_onc_rot = dimensions[2]    # rotation dimensions
    D_onc_other = dimensions[3]  # other dimensions of ontic state
    D_onc = D_onc_pos + D_onc_rot + D_onc_other  # total dimensions ontic state
    D_epi = dimensions[4]       # total dimensions epistemic state

    @assert D_onc == length(ranges)  # make sure all variables have ranges 
    @assert D_onc_pos == length(extent)  # make sure there are enough coordinates

    properties = Dict(  # save the time step in the model
        :δt => δt,
        :history_size => history_size,
        :colors => distinguishable_colors(N, [RGB(1,1,1), RGB(0,0,0)], dropseed=true),
        :seed => seed,
        :speed => speed,
       )

    model = AgentBasedModel(Robot{D_onc, D_epi}, space;
                rng = rng,
                scheduler = scheduler,
                properties = properties
               )

    # now we add the agents
    for n ∈ 1:N
        #initialize ontic state
#        x = @SVector [rand(model.rng) * (range[i][2] - range[i][1]) + range[i][1] for i = 1:D_onc]

        x = ntuple(i -> rand(model.rng) * (range[i][2] - range[i][1]) + range[i][1], D_onc)
        dx = ntuple(i->0, D_onc)

        vel = speed.* (
                       x[1]*cos(x[D_onc_pos+1]) - x[2]*sin(D_onc_pos+1),
                       x[1]*sin(x[D_onc_pos+1]) + x[2]*cos(D_onc_pos+1)
                      )

        # initialize epistemic state to 0 or copy from ontic
        μ = ntuple(i -> i ≤ copy_ontic ? x[i] : 0, D_epi)
        dμ = ntuple(i -> i ≤ copy_ontic ? dx[i] : 0, D_epi)

        # initialize the agents with argument values and no heading change
        agent = Robot{D}(n, x, dx, μ, dμ, vis_range, com_range, true)
        add_agent!(agent, model)
    end

    return model
end

# we define how each robot behaves, at
function agent_laplacian_step!(r1, model)
    neighbours = nearby_agents(r1, model, r1.vis_range)
    σ = 1.0

    dpos = r1.pos
    for r2 ∈ neighbours
        dpos = dpos .- σ .* (r1.pos .- r2.pos)
    end

    r1.dθ = 0
    r1.vel = dpos

    r1.pos = r1.pos .+ r1.vel .* model.δt
    r1.θ = atan(r1.vel[2], r1.vel[1])

    return
end

# a sim behaviour for the robots
# - they keep a low speed with 0 rotation speed
# - unless they reach further than X meters from the nearest neighbor,
# then they rotate
function agent_simple_step!(robot, model)
    neighbor_ids = nearby_ids(robot, model, robot.vis_range)

    robot.pos = robot.pos .+ robot.vel .* model.δt
    robot.θ += robot.dθ * model.δt

    robot.vel = (
        robot.pos[1]*cos(robot.θ) - robot.pos[2]*sin(robot.θ),
        robot.pos[1]*sin(robot.θ) + robot.pos[2]*cos(robot.θ)
    )

    for id in neighbor_ids
        if euclidean_distance(robot.pos, model[id].pos, model) > robot.com_range
            robot.dθ += 0.01
            return
        end
    end

    robot.dθ -= 0.005

    return
end

end # module

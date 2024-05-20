using DrWatson
@quickactivate "Essaim"

include(srcdir("utils.jl"))

using Agents
using DataStructures: CircularBuffer
using Random
using Colors

mutable struct Robot{D,O,E} <: AbstractAgent
    id::Int

    # pos and vel are needed like this for ContinuousSpace struct
#    pos::SVector{D,Float64}  # position part of ontic state
#    vel::SVector{D,Float64}  # velocity ...
#
#    x::SVector{O,Float64}   # ontic state
#    dx::SVector{O,Float64}  # d/dt ontic state
#
#    μ::SVector{O,Float64}   # epistemic state
#    dμ::SVector{O,Float64}  # d/dt epistemic state
    pos::NTuple{D,Float64}  # position part of ontic state
    vel::NTuple{D,Float64}  # velocity ...

    x::NTuple{O,Float64}   # ontic state
    dx::NTuple{O,Float64}  # d/dt ontic state

    μ::NTuple{E,Float64}   # epistemic state
    dμ::NTuple{E,Float64}  # d/dt epistemic state

    vis_range::Float64  # perception range
    com_range::Float64  # communication range
    alive::Bool  # is it alive?
end

# here we initialize the model with its parameters and populate with robots
function initialize_model(;
                          N = 10,                  # number of agents
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
    D = dimensions[1]            # position dimensions
    @assert D == length(extent)  # make sure there are enough coordinates
    @assert 1 <= D && D <= 3     # it only makes sense to have robots in 1d, 2d or 3d

    O_rot = dimensions[2]    # rotation dimensions
    O_other = dimensions[3]  # other dimensions of ontic state
    O = D + O_rot + O_other  # total dimensions ontic state
    @assert O == length(range_dims) # make sure all ontic state variables have range

    E = dimensions[4]        # total dimensions epistemic state


    properties = Dict(  # save the time step in the model
        :δt => δt,
        :history_size => history_size,
        :colors => distinguishable_colors(N, [RGB(1,1,1), RGB(0,0,0)], dropseed=true),
        :seed => seed,
        :speed => speed,
       )

    model = AgentBasedModel(Robot{D, O-D, E}, space;
                rng = rng,
                scheduler = scheduler,
                properties = properties
               )

    # now we add the agents
    for n ∈ 1:N
        #initialize ontic state
        pos_v = SVector{D}([rand(model.rng) * (range_dims[i][2] - range_dims[i][1]) + range_dims[i][1] for i ∈ 1:D])
#
#        rot = (D == 1 ? 1 : # if D == 1 there is no rotation
#               (D == 2 ? rot2D(x[1]) :
#                rot3D(x[1], x[2], x[3])))
#        vel = speed * rot * pos
#
#        # everything ontic that is not the coordinates or their derivative
#        x = SVector{O-D}([rand(model.rng) * (range[i][2] - range[i][1]) + range[i][1] for i ∈ D+1:O])
#        dx = SVector{O-D}([0 for i ∈ D+1:O])
#
#        # initialize epistemic state to 0 or copy from ontic
#        μ = SVector{E}([(i ≤ copy_ontic ? x[i] : 0) for i ∈ 1:E])
#        dμ = SVector{E}([(i ≤ copy_ontic ? x[i] : 0) for i ∈ 1:E])
        pos = ntuple(i -> rand(model.rng) * (range_dims[i][2] - range_dims[i][1]) + range_dims[i][1], D)
        x   = ntuple(i->rand(model.rng) * (range_dims[D+i][2] - range_dims[D+i][1]) + range_dims[D+i][1], O-D)
        dx  = ntuple(i -> O, O-D)

        rot = (D == 1 ? 1 : # if D == 1 there is no rotation
               (D == 2 ? rot2D(x[1]) :
                rot3D(x[1], x[2], x[3])))
        vel_v = speed * rot * pos_v

        vel = ntuple(i -> vel_v[i], D)

        # initialize epistemic state to 0 or copy from ontic
        μ  = ntuple( i -> i ≤ copy_ontic && i ≤ D ? pos[i] : (i ≤ copy_ontic && i > D ? x[i-D] : 0), E)
        dμ = ntuple( i -> i ≤ copy_ontic && i ≤ D ? vel[i] : (i ≤ copy_ontic && i > D ? dx[i-D] : 0), E)

        # initialize the agents with argument values and no heading change
        agent = Robot{D,O-D,E}(n, pos, vel, x, dx, μ, dμ, vis_range, com_range, true)
        add_agent!(agent, model)
    end

    return model
end

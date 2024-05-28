using DrWatson
@quickactivate "Essaim"

include(srcdir("utils.jl"))

using Agents
using DataStructures: CircularBuffer
using Random
using Colors

mutable struct OnticStateDeriv{O}
    x::MVector{O,Float64}   # ontic state
    dx::MVector{O,Float64}  # d/dt ontic state
end

mutable struct EpistemicStateDeriv{E}
    μ::MVector{E,Float64}   # epistemic state
    dμ::MVector{E,Float64}  # d/dt epistemic state
end

mutable struct Robot{D,OnticState,EpistemicState} <: AbstractAgent
    id::Int

    # pos and vel are needed like this for ContinuousSpace functions
    pos::SVector{D,Float64}  # position part of ontic state
    vel::SVector{D,Float64}  # velocity ...
    onc::OnticState          # whatever is the ontic state
    epi::EpistemicState      # whatever is the epistemic state

    vis_range::Float64  # perception range
    com_range::Float64  # communication range
    alive::Bool  # is it alive?
end

# here we initialize the model with its parameters and populate with robots
function initialize_model(agent_input_step!;
                          N = 3,                  # number of agents
                          ontic_state = OnticStateDeriv,
                          epistemic_state = EpistemicStateDeriv,
                          dimensions = (2,             # dim ontic position
                                        1,             # dim ontic rotation
                                        0,             # dim other ontic
                                        2),            # dim epi state
                          range_dims = ((25.0, 75.0),  # range pos x
                                        (25.0, 75.0),  # range pos y
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
    O = O_rot + O_other      # total dimensions ontic state (without coordinates!)
    @assert O + D == length(range_dims) # make sure all ontic state variables have range

    E = dimensions[4]        # total dimensions epistemic state

    properties = Dict(  # save the time step in the model
        :dimensions => dimensions,
        :range_dims => range_dims,
        :copy_ontic => copy_ontic,
        :ontic_state => ontic_state{O},
        :epistemic_state => epistemic_state{E},
        :step_function! => agent_input_step!,
        :δt => δt,
        :history_size => history_size,
        :colors => distinguishable_colors(N, [RGB(1,1,1), RGB(0,0,0)], dropseed=true),
        :seed => seed,
        :speed => speed,
       )

    # WARN: StandardABM works in discrete time, use EventQueueABM for continuous
    
    # model = StandardABM(Robot{D, O-D, E}, space;
    model = StandardABM(Robot{D, ontic_state{O}, epistemic_state{E}}, space;
                rng = rng,
                scheduler = scheduler,
                properties = properties,
                agent_step! = agent_input_step!,
                container = Vector,  #WARN: Using vector because no agents are 
                                     # expected to be removed, change to Dict
                                     # otherwise
                agents_first = false
               )

    # now we add the agents
    for n ∈ 1:N
        #initialize ontic state
        pos = SVector{D}([rand(abmrng(model)) * (range_dims[i][2] - range_dims[i][1]) + range_dims[i][1] for i ∈ 1:D])

        # everything ontic that is not the coordinates or their derivative
        x = MVector{O}([rand(abmrng(model)) * (range_dims[i][2] - range_dims[i][1]) + range_dims[i][1] for i ∈ D+1:D+O])
        dx = MVector{O}([0 for i ∈ D+1:D+O])

        onc = ontic_state{O}(x, dx)

        # compute velocity from rotation
        rot = (D == 1 ? 1 : # if D == 1 there is no rotation
               (D == 2 ? rot2D(x[1]) :
                rot3D(x[1], x[2], x[3])))
        vel = speed * rot * pos

        # initialize epistemic state to 0 or copy from ontic
        # get the first D values from pos and vel, the rest from x and dx
        μ = MVector{E}([(i ≤ copy_ontic && i ≤ D) ? pos[i] : (i ≤ copy_ontic && i > D ? x[i-D] : 0) for i ∈ 1:E])
        dμ = MVector{E}([(i ≤ copy_ontic && i ≤ D) ? vel[i] : (i ≤ copy_ontic && i > D ? dx[i-D] : 0) for i ∈ 1:E])

        epi = epistemic_state{E}(μ, dμ)

        # initialize the agents with argument values and no heading change
        agent = Robot{D,ontic_state{O},epistemic_state{E}}(n, pos, vel, onc, epi, vis_range, com_range, true)

        add_agent_own_pos!(agent, model)  # WARN: Use this version to preserve position
    end

    return model
end

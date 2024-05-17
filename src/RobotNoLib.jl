struct Robot
  x::Float64
  y::Float64
  θ::Float64
end

# maybe in the future, without history but live simulation
# Base.@kwdef mutable struct Essaim
#     dt::Float64 = 0.01
#     R::Vector{Robot}
#     Ṙ::Vector{Robot}
#     A::Matrix{Int64}
#
# end

# received a robot and returns the edges of the polygon representing it
struct Configuration
  R::Vector{Robot}
  Ṙ::Vector{Robot}
  A::Matrix{Int64}
end

# control based on the dubin's car model
function d_dubins(X::Robot, u::Vector{Float64})::Robot
  ẋ = u[1] * cos(X.θ)
  ẏ = u[1] * sin(X.θ)
  θ̇ = u[2]
  Robot(ẋ, ẏ, θ̇)
end

function c_constant(X::Robot)
  u = [1.0, 0.1]
end

# evolve each robot by euler integration
# requires:
#   robot: the robot to evolve
#   Δt: the time step
#   deriv: the derivative function
function evolve(X::Robot, Ẋ::Robot, δt::Float64,)::Robot
  x = X.x + δt * Ẋ.x
  y = X.y + δt * Ẋ.y
  θ = X.θ + δt * Ẋ.θ
  Robot(x, y, θ)
end

function step(config::Configuration, deriv::Function, u::Vector{Vector{Float64}}, δt::Float64)::Configuration
    R_t = config.R
    Ṙ_t = config.Ṙ
    A_t = config.A

    N = length(R_t)

    # generate derivative
    Ṙ_new = Vector{Robot}(undef, N)
    for i ∈ 1:N
      Ṙ_new[i] = deriv(R_t[i], u[i])
    end

    # evolve robots
    R_new = Vector{Robot}(undef, N)
    for i in 1:N
      R_new[i] = evolve(R_t[i], Ṙ_new[i], δt)
    end

    A_new = A  # say that visibility never changes

    config_new = Configuration(R_new, Ṙ_new, A_new)
    config_new
end

"""
  Input: 
  - a list of robots in their initial state
  - a visibility matrix, 
  - a derivative function
  -  a control function 
  Output:
  - the execution of the mission.
"""
function execute_mission(R::Array{Robot,1}, A::Array{Int64,2}, deriv::Function, control::Function)::Vector{Configuration}
  # number of robot
  N = length(R)

  # robots
  execution = Array{Configuration}(undef, 0)
  config = Configuration(R, [Robot(0, 0, 0) for robot in R], A)
  push!(execution, config)

  δt = 0.1 # time step
  t = 0.0 
  tf = 10.0

  # generate inputs
  u = [control(R[i]) for i ∈ 1:N]

  while t < tf
    config_new = step(execution[end], deriv, u, δt)

    push!(execution, config_new)
    t = t + δt
  end

  execution
end

using DrWatson
@quickactivate "Essaim"

using DynamicalSystems

# (1) Discrete example

# most dynamical agents have
# - a dynamical rule: f
# - a state u
# (the results of the evolution function and the observation function)
# - a set of parameters for f: p
function henon_rule(u, p, n)
    x, y = u
    a, b = p
    xn = 1.0 - a*x^2 + y
    yn = b*x

    # SVector is a static vector
    return SVector(xn, yn)
end

u0 = [0.2, 0.3]
p0 = [1.4, 0.3]

henon = DeterministicIteratedMap(henon_rule, u0, p0)

total_time = 10_000
X, y = trajectory(henon, total_time)

using GLMakie

fig = Figure()
ax = Axis(fig[1,1])
scatter!(ax, X[:,1], X[:,2])


# (2) Continuous example
# the function being in place, we have the derivative of the state as the first argument
function lorenz96_rule!(du, u, p, t)
    F = p[1]
    N = length(u)
    # 3 edge cases
    du[1] = (u[2] - u[N - 1]) * u[N] - u[1] + F
    du[2] = (u[3] - u[N]) * u[1] - u[2] + F
    du[N] = (u[1] - u[N - 2]) * u[N - 1] - u[N] + F
    # then the general case
    for n in 3:(N - 1)
        du[n] = (u[n + 1] - u[n - 2]) * u[n - 1] - u[n] + F
    end
    return nothing # always `return nothing` for in-place form!
end

N = 6
u0 = range(0.1, 1; length = N)
p0 = [8.0]

lorenz96 = CoupledODEs(lorenz96_rule!, u0, p0)

total_time = 12.5
sampling_time = 0.02
Y, t = trajectory(lorenz96, total_time; Ttr = 2.2, Δt = sampling_time)
Y

# what i can change is the parameter p, this could maybe be the input from the other robots

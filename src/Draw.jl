using DrWatson
@quickactivate "Essaim"

include(srcdir("Essaim.jl"))

# using Essaim: Robot, execute_mission, d_dubins, c_constant
using Plots
plotlyjs()

# receive a robot and returns the edges of the polygon representing it
function triangle(X::Essaim.Robot, base, height)::Matrix{Float64}
  # base triangle
  P::Matrix{Float64} = [
     height/2       0
    -height/2  base/2
    -height/2 -base/2
  ]

  # first rotate p' = R * p
  c::Float64, s::Float64 = cos(X.θ), sin(X.θ)
  P = [
    c -s
    s  c
  ] * P'

  # then translate p = p .+ x
  P = P' .+ [X.x X.y]

  P
end

function triangle_shape(triangle)
  t = vcat(triangle, triangle[1,:]')
  shape = [
    (t[i,:][1],t[i,:][2]) for i in axes(t,1)]

  shape
end

function draw_mission(mission, base_plot, filename, fps=5, triangle_size=(0.5,0.5))
  # triangle parameters
  base, height = triangle_size

  # start animation
  anim = Animation()

  for idx_m ∈ eachindex(mission[2:end])
    plt = plot(base_plt) # clear the plot
    colors = ["blue", "red", "green"]
    alpha = range(0,1,length=idx_m+1)

    for i ∈ 1:idx_m

      N = length(mission[1].R) # number of robots

      triangles = [triangle(robot, base, height) for robot ∈ mission[i].R]
      shapes = [triangle_shape(triangle) for triangle ∈ triangles]

      for r ∈ 1:N
        plot!(Shape(shapes[r]), label="Robot {$r}",
         fillcolor = if i == idx_m colors[r] else "white" end,
         linewidth = if i == idx_m 1 else 0.3 end,
         fillalpha = alpha[i],
         aspect_ratio = :equal
         )
      end
    end

    frame(anim, plt)
  end

  # generate animation
  gif(anim, plotsdir(filename), fps=5)
end

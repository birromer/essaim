using DrWatson
@quickactivate "Essaim"

include(srcdir("Essaim.jl"))
include(srcdir("Draw.jl"))

# initialize plot
base_plt = plot(
  1,  # 1 empty series
  xlim = (-10, 20),
  ylim = (-10, 10),
  title = "Dynamical Laplacian Average Consensus",
  legend = false,
  aspect_ration = :equal
)

# visibility matrix
A = [
  0 1 1
  1 0 1
  1 1 0
]

# list of robots
R = [
  Robot(0,0,0),
  Robot(0,1,0),
  Robot(1,0,0)
]

# run mission
mission = execute_mission(R, A, d_dubins, c_constant)

N = length(R) # number of robots
len = length(mission) # lenght of the mission

# triangle parameters
base = 0.5
height = 0.5

# generate animation
draw_mission(mission, base_plt, "anim_fps5.gif", 5, (base,height))

using LinearAlgebra

# System and task specifications
include("System.jl")
include("Task.jl")

# Pipeline procedures
include("SystemSlice.jl")
include("TaskSheaf.jl")
include("Linearize.jl")
include("Derive.jl")
include("Decide.jl")

function run_approx_agreement(k, epsilon)
  frame  = system_frame(system_data(), k)
  slice  = system_slice(frame)
  sheaf  = build_sheaf(slice, task_data(epsilon))
  D      = coboundary(sheaf)
  H0     = derive(D)
  decide(H0, sheaf)
end

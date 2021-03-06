module GraphSearchZero
using DataStructures

export SearchProblem
# Describes how problems need to be specified in order to work with this module.
include("problem_interface.jl")
# A simple but fairly fast implementation of a search node.
export SearchNode
include("search_node.jl")
# Implementations of some graphs search algorithms.
export
    InfeasibleSearchProblemError,
    generic_graph_search,
    astar_search,
    weighted_astar_search
include("search_algorithms.jl")

end # module

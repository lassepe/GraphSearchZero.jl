"""
    SearchProblem

A minimalistic, simple interface for search problems
`S` - the state type
`A` - the action type
"""
abstract type SearchProblem{S, A} end
"""
    start_state(p::SearchProblem)

Returns the start state of the problem.
"""
function start_state end
"""
    is_goal_state(p::SearchProblem, s::S)

Returns true if a given state is a goal state
"""
function is_goal_state end
"""
    successors(p::SearchProblem, s::S)

Returns a list of tuples `(sp, a, c)`, i.e. the next state, the action that
yields this state and the cost for this transition.
"""
function successors end
"""
    search_node_type

Returns the SearchNode parametrized according to the SearchProblem
"""
search_node_type(::SearchProblem{S, A}) where {S, A} = SearchNode{S, A}
"""
    state_type

Returns the type of the state the search is run over.
"""
state_type(::SearchProblem{S, A}) where {S, A} = S
"""
    action_type

Returns the type of the action the search is run over.
"""
action_type(::SearchProblem{S, A}) where {S, A} = A

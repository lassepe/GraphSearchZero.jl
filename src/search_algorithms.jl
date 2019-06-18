struct InfeasibleSearchProblemError <: Exception
    msg::String
end

function generic_graph_search(p::SearchProblem, fringe_priority::Function)
    # the closed set, states that we don't need to expand anymore
    closed_set = Set{state_type(p)}()
    # the fringe is a priority queue of SearchNode that are left to be expanded
    fringe = PriorityQueue{search_node_type(p), Float64}()

    n0 = root_node(p)
    enqueue!(fringe, n0, fringe_priority(n0))
    while true
        if isempty(fringe)
            throw(InfeasibleSearchProblemError("Fringe was empty, but no solution found"))
        end
        current_search_node = dequeue!(fringe)
        # We have found a path to the goal state
        if is_goal_state(p, leaf_state(current_search_node))
            return action_sequence(current_search_node), state_sequence(current_search_node), cost(current_search_node)
        elseif !(leaf_state(current_search_node) in closed_set)
            # make sure we don't explore this state again
            push!(closed_set, leaf_state(current_search_node))
            # expand the node
            for child_search_node in expand(current_search_node, p)
                enqueue!(fringe, child_search_node, fringe_priority(child_search_node))
            end
        else
            continue
        end
    end
end

astar_search(p::SearchProblem, heuristic::Function) = weighted_astar_search(p, heuristic, 0.0)

function weighted_astar_search(p::SearchProblem, heuristic::Function, eps::Float64)
    @assert eps >= 0.0
    astar_priority = (n::SearchNode) -> cost(n) + (1+eps)*heuristic(leaf_state(n))
    return generic_graph_search(p, astar_priority)
end

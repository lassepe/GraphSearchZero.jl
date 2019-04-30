"""
    NodeRegistry

A registry to keep track of the node indeces already assigned. Used to speed up
hashing.
"""
mutable struct NodeRegistry
    latest_id::Int
end
"""
generate_next_id!

Generates the next, unassigned id and updates the internal state.
"""
function generate_next_id!(nr::NodeRegistry)
    nr.latest_id += 1
    return nr.latest_id
end
"""
    SearchNode

Describes a path along the graph. Containting states, action and cost.
"""
struct SearchNode{S, A}
    "The  state at the end of the path represented by this node"
    leaf_state::S
    "The  action at the end of the path represented by this node"
    leaf_action::Union{A, Nothing}
    "The cumulative cost for traversing the state trajectory represented by this node."
    cost::Float64
    "The parent node, if not a root node"
    parent::Union{SearchNode{S, A}, Nothing}
    "The depth of this node in the tree."
    depth::Int
    "A node registry that can generate unique identifiers"
    node_registry::NodeRegistry
    "The identifier of this node"
    id::Int
end
"""
    state_type

Returns the type of the state the search is run over.
"""
state_type(::SearchNode{S, A}) where {S, A} = S
"""
    action_type

Returns the type of the action the search is run over.
"""
action_type(::SearchNode{S, A}) where {S, A} = A
"""
    depth(n::SearchNode)

The depth of the search node in the tree.
"""
depth(n::SearchNode) = n.depth
"""
    Base.hash

Computes a fast hash for a search node.
"""
Base.hash(n::SearchNode) = n.id
"""
    Base.:==

Fast comparison for SearchNode based on id.
"""
==(n1::SearchNode, n2::SearchNode) = n1.id == n2.id
"""
    parent(n::SearchNode)

The parent node of the given search node. `nothing` if n is a root node.
"""
parent(n::SearchNode) = n.parent
"""
    cost(n::SearchNode)

Returns the cost for traversing the path described by the node.
"""
cost(n::SearchNode) = n.cost
"""
    leaf_state(n::SearchNode)

Returns the last state of the path described by the node.
"""
leaf_state(n::SearchNode) = n.leaf_state
"""
    leaf_action(n::SearchNode)

Returns the last action on the path described by this node.
"""
leaf_action(n::SearchNode) = n.leaf_action
"""
    action_sequence(n::SearchNode)

Returns the action sequence neccessary to traverse the path to the `leaf_state`
"""
function action_sequence(n::SearchNode)
    action_sequence = action_type(n)[]
    resize!(action_sequence, depth(n))

    current_node = n
    # traversing the path of the node in reverse order
    for i in reverse(1:depth(n))
        action_sequence[i] = leaf_action(current_node)
        current_node = parent(current_node)
    end
    # at the end of this path there must be a root node.
    # The parent of a root node is nothing!
    @assert isnothing(parent(current_node))

    return action_sequence::Vector{action_type(n)}
end
"""
    state_sequence(n::SearchNode)

Returns the sequence of states along the path represented by this node
"""
function state_sequence(n::SearchNode)
    state_sequence = state_type(n)[]
    resize!(state_sequence, depth(n))

    current_node = n
    # traversing the path of the node in reverse order
    for i in reverse(1:depth(n))
        state_sequence[i] = leaf_state(current_node)
        current_node = parent(current_node)
    end
    # at the end of this path there must be a root node.
    # The parent of a root node is nothing!
    @assert isnothing(parent(current_node))

    return state_sequence::Vector{state_type(n)}
end
"""
    expand(n::SearchNode)

Returns the child search nodes (set of next, longer pathes from this node)
"""
function expand(n::SearchNode, p::SearchProblem)
    # TODO: maybe resize in advance or sizehint!
    child_search_nodes = search_node_type(p)[]
    for (sp, a, c) in successors(p, leaf_state(n))
        np = SearchNode(sp, a, cost(n) + c, n, depth(n)+1, n.node_registry, generate_next_id!(n.node_registry))
        push!(child_search_nodes, np)
    end

    return child_search_nodes
end
"""
    root_node

Returns a SearchNode over the start_state of this problem
"""
function root_node(p::SearchProblem, root_cost::Float64=0.0)
    s = start_state(p)
    a = nothing
    c = root_cost
    parent = nothing
    depth = 0
    node_registry = NodeRegistry(0)
    id = generate_next_id!(node_registry)
    return search_node_type(p)(s, a, c, parent, depth, node_registry, id)
end

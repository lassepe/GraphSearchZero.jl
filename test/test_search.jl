using Parameters

# defining a simple test search problem
struct GridPosition
    x_idx::Int
    y_idx::Int
end

struct GridAction
    dx::Int
    dy::Int
end

apply_action(s::GridPosition, a::GridAction) = GridPosition(s.x_idx + a.dx, s.y_idx + a.dy)

@with_kw mutable struct GridNavigationProblem <: SearchProblem{GridPosition, GridAction}
    grid_dimensions::Tuple{Int, Int} = (10, 10)
    obstacles::Vector{GridPosition} = []
    aspace::Vector{GridAction} = [GridAction(0, 0),
                                  GridAction(1, 0), GridAction(-1, 0),
                                  GridAction(0, 1), GridAction(0, -1)]
    _n_expanded::Int = 0
end

GraphSearchZero.start_state(p::GridNavigationProblem) = GridPosition(1, 1)
GraphSearchZero.is_goal_state(p::GridNavigationProblem, s::GridPosition) = s == GridPosition(p.grid_dimensions...)
on_grid(p::GridNavigationProblem, s::GridPosition) = (1 <= s.x_idx <= p.grid_dimensions[1]) && (1 <= s.y_idx <= p.grid_dimensions[2])

function GraphSearchZero.successors(p::GridNavigationProblem, s::GridPosition)
    p._n_expanded += 1
    successors::Vector{Tuple{GridPosition, GridAction, Int}} = []
    sizehint!(successors, length(p.aspace))
    for a in p.aspace
        sp = apply_action(s, a)
        if !on_grid(p, sp) || sp in p.obstacles
            continue
        end
        # we are solving a minimum time problem, the step cost is always 1
        push!(successors, (sp, a, 1))
    end
    return successors
end

@testset "SearchTest" begin
    # tuples of (dimensions, obstacles, optimal_nsteps, solvable, n_expanded)
    test_setups::Vector{Tuple{Tuple{Int, Int}, Vector, Union{Int, Nothing}, Bool, Union{Int, Nothing}}} =
    [
     # 1x1 grid, no action needed to reach the goal
     ((1, 1), [], 0, true, 0),
     # empty 10x10 grid, solvable in 18 steps
     ((10, 10), [], 18, true, 18),
     # 10x10 grid with wall in the middle that has a gap, solvable in 18 steps
     ((10, 10), [GridPosition(5, y) for y in 1:9], 18, true, 18),
     # 10x10 grid with wall all the way, not solvable
     ((10, 10), [GridPosition(5, y) for y in 1:10], nothing, false, 40),
     # 5x5 grid with two walls, gap at top and bottom, solvable in 16 steps
     ((5, 5), [[GridPosition(2, y) for y in 1:4]...,
               [GridPosition(4, y) for y in 2:5]...], 16, true, 16)
    ]

    for (test_dims, test_obstacles, optimal_nsteps, solvable, n_expanded) in test_setups

        p = GridNavigationProblem(grid_dimensions=test_dims,
                                  obstacles=test_obstacles)
        # using the manhattan distance as a heuristic
        h = (s::GridPosition) -> abs(s.x_idx - p.grid_dimensions[1]) + abs(s.y_idx - p.grid_dimensions[2])
        eps = 0.0001
        if solvable
            aseq, sseq = @inferred weighted_astar_search(p, h, eps)
            @test length(aseq) == optimal_nsteps
        else
            @test_throws InfeasibleSearchProblemError weighted_astar_search(p, h, eps)
        end
        @test p._n_expanded == n_expanded


        # also run type inference tests:
        if solvable
            @test @testblock quote
                pp = deepcopy(p)
                a = rand(pp.aspace)
                n = GraphSearchZero.root_node(pp)
                @inferred GraphSearchZero.expand(n, pp)
            end
        end
    end
end


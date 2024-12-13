mutable struct Subproblem
    instance::Instance

    pi::Vector{Float64}

    picost::Matrix{Float64}

    ng::Matrix{Int}

    gap::Float64

    tbounds::Array{Float64,3}
end

function Subproblem(instance::Instance)
    n = n_nodes(instance)

    pi = zeros(Float64, n)

    picost = zeros(Float64, n, n)

    ng = zeros(Int, delta - 1, n)

    tbounds = [Inf for _ in 1:128, _ in 1:n, _ in 1:tmax(instance)]

    return Subproblem(instance, pi, picost, ng, 0.0, tbounds)
end

@inline n_nodes(sp::Subproblem) = n_nodes(sp.instance)
@inline root(sp::Subproblem) = root(sp.instance)
@inline vehicle_capacity(sp::Subproblem) = vehicle_capacity(sp.instance)
@inline vehicle_max_travel_time(sp::Subproblem) = vehicle_max_travel_time(sp.instance)

@inline tmin(sp::Subproblem) = tmin(sp.instance)
@inline tmax(sp::Subproblem) = tmax(sp.instance)
@inline soft_distance_limit(sp::Subproblem) = soft_distance_limit(sp.instance)
@inline hard_distance_limit(sp::Subproblem) = hard_distance_limit(sp.instance)

@inline request_twstart(sp::Subproblem, i::Int) = request_twstart(sp.instance, i)
@inline request_twend(sp::Subproblem, i::Int) = request_twend(sp.instance, i)

@inline request_quantity(sp::Subproblem, i::Int) = request_quantity(sp.instance, i)
@inline request_service_time(sp::Subproblem, i::Int) = request_service_time(sp.instance, i)

@inline distance(sp::Subproblem, i::Int, j::Int) = distance(sp.instance, i, j)
@inline traveltime(sp::Subproblem, i::Int, j::Int) = traveltime(sp.instance, i, j)
@inline picost(sp::Subproblem, i, j) = @inbounds sp.picost[i, j]
@inline adjacent(sp::Subproblem, i::Int, j::Int) = @inbounds sp.picost[i, j] < Inf

@inline backward_picost(sp::Subproblem, i, j) = @inbounds sp.picost[j, i]
@inline backward_traveltime(sp::Subproblem, i::Int, j::Int) = backward_traveltime(sp.instance, i, j)
@inline backward_distance(sp::Subproblem, i::Int, j::Int) = backward_distance(sp.instance, i, j)
@inline backward_twstart(sp::Subproblem, i::Int) = tmax(sp) - request_twend(sp, i)
@inline backward_twend(sp::Subproblem, i::Int) = tmax(sp) - request_twstart(sp, i)
@inline backward_adjacent(sp::Subproblem, i::Int, j::Int) = @inbounds adjacent(sp, j, i)

@inline resetpi(sp::Subproblem) = sp.pi .= 0.0

function set(sp::Subproblem, constraints::Vector{ConstraintRef})
    n = n_nodes(sp)

    for i in 1:n
        sp.pi[i] = dual(constraints[i])
    end

    for i in 1:n
        for j in 1:i-1
            sp.picost[i, j] = -sp.pi[i] / 2.0 - sp.pi[j] / 2.0
        end

        sp.picost[i, i] = Inf

        for j in i+1:n
            sp.picost[i, j] = -sp.pi[i] / 2.0 - sp.pi[j] / 2.0
        end
    end
end

# TODO: question comment on fait pour les ng ? distance, reduced cost, etc
function build_ng(sp::Subproblem)
    n = n_nodes(sp)

    for i in 2:n
        # TODO: modifier ici
        adj = [j for j in 2:n if j != i]

        # TODO: une combinaison des deux ? mais cela n'a un sens que si on dépasse la soft bound
        sort!(adj, by=x -> adjacent(sp, i, x) ? distance(sp, i, x) + sp.picost[i, x] : Inf)

        for j in 1:delta-1
            @inbounds sp.ng[j, i] = adj[j]
        end

        sort!(view(sp.ng, :, i))
    end
end

@inline set_branching_0(sp::Subproblem, u::Int, v::Int) = sp.picost[u, v] = Inf

function set_branching_1(sp::Subproblem, u::Int, v::Int)
    for i in 1:n_nodes(sp)
        if i == u || i == v
            continue
        end

        sp.picost[u, i] = Inf
        sp.picost[i, v] = Inf
    end
end

function set_branching(sp::Subproblem, branchments::Vector{Tuple{Int,Int,Int}})
    for (u, v, value) in branchments
        if value == 0
            set_branching_0(sp, u, v)
        elseif value == 1
            set_branching_1(sp, u, v)
        end
    end
end

function check_cost(sp::Subproblem, route::Vector{Int})
    cost = 0

    for i in 1:length(route)-1
        cost += distance(sp, route[i], route[i+1])
    end

    return cost
end

function compute_rc(sp::Subproblem, route::Vector{Int})
    rc = 0.0

    for i in 1:length(route)-1
        rc += distance(sp, route[i], route[i+1]) + picost(sp, route[i], route[i+1])
    end

    return rc
end

function compatible(sp, node, ng::UInt8, u::BitSet)
    pos = 1

    while pos <= 7 && ng != 0
        if ng & 1 == 1 && in(sp.ng[pos, node], u)
            return false
        end

        ng >>= 1
        pos += 1
    end

    return true
end

# TODO: A implementer -> retirer le typage
function search_bound(sp, node, load, u::BitSet)
    qbounds = sp.qbounds
    best_bound = qbounds[1, node, load]

    for ng in 2:128
        # TODO: est-ce que le sous-ensemble ng-1 intersection u est vide ?
        if compatible(sp, node, UInt8(ng - 1), u)
            continue
        end

        if qbounds[ng, node, load] < best_bound
            best_bound = qbounds[ng, node, load]
        end
    end

    return best_bound
end
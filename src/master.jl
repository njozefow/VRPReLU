mutable struct Master
    model::Model

    x::Vector{VariableRef} # elementary routes
    columns::Vector{Column}

    ngx::Vector{VariableRef} # non elementary routes
    ngcolumns::Vector{Column}

    artificial_x::VariableRef
    artificial_x_present::Bool

    constraints::Vector{ConstraintRef}
end

@inline objective_value(master::Master) = JuMP.objective_value(master.model)
@inline status(master) = termination_status(master.model)

# TODO: add stabilization
function Master(instance)
    model = direct_model(Gurobi.Optimizer(GRB_ENV_REF[]))
    set_silent(model)
    set_string_names_on_creation(model, false)
    set_attribute(model, "Threads", 1) # If gurobi is used

    @variable(model, 0 <= artificial_x)
    artificial_x_present = true

    @objective(model, Min, artificial_weight(instance) * artificial_x)

    constraints = Vector{ConstraintRef}()
    push!(constraints, @constraint(model, n_vehicles(instance) * artificial_x == n_vehicles(instance), set_string_name = false))
    for i in 2:n_nodes(instance)
        push!(constraints, @constraint(model, artificial_x == 1, set_string_name = false))
    end

    return Master(model, Vector{VariableRef}(), Vector{Column}(), Vector{VariableRef}(), Vector{Column}(), artificial_x, artificial_x_present, constraints)
end

function switch_to_integer(master)
    for x in master.x
        set_binary(x)
    end
end

function optimize(master, timelimit=3600.0)
    set_time_limit_sec(master.model, timelimit)
    optimize!(master.model)

    if termination_status(master.model) != MOI.OPTIMAL
        set_upper_bound(master.artificial_x, 1.0)
        master.artificial_x_present = true
        optimize!(master.model)
    elseif master.artificial_x_present && value(master.artificial_x) < 1e-4
        set_upper_bound(master.artificial_x, 0.0)
        master.artificial_x_present = false
        optimize!(master.model)
    end
end

function iselemantary(route)
    elementary = true
    counter = Dict{Int,Int}()

    @inbounds for u in view(route, 2:length(route)-1)
        if haskey(counter, u)
            counter[u] += 1
            elementary = false
        else
            counter[u] = 1
        end
    end

    return elementary, counter
end

# TODO: il faudrait le faire en construisant column
function push_column(master, column)
    elementary, counter = iselemantary(column.route)

    x = elementary ? master.x : master.ngx

    push!(x, @variable(master.model, lower_bound = 0.0))
    set_objective_coefficient(master.model, x[end], column.cost)

    set_normalized_coefficient(master.constraints[1], x[end], 1.0)
    for (u, c) in counter
        set_normalized_coefficient(master.constraints[u], x[end], c)
    end

    push!(elementary ? master.columns : master.ngcolumns, column)
end

# TODO: A reprendre pour simplifier
function push_elementary_column(master, columns)
    constraints = master.constraints
    model = master.model

    for c in columns
        x = @variable(model, lower_bound = 0.0, integer = true, set_string_name = false)
        set_objective_coefficient(model, x, c.cost)
        @inbounds set_normalized_coefficient(constraints[1], x, 1.0)
        for u in view(c.route, 2:length(c.route)-1)
            @inbounds set_normalized_coefficient(constraints[u], x, 1.0)
        end

        push!(master.x, x)
        push!(master.columns, c)
    end
end

function delete_ngroutes(master)
    for x in master.ngx
        delete(master.model, x)
    end
end

function add_singleton(master, instance)
    for i in 2:n_nodes(instance)
        length = 2 * distance(instance, root(instance), i)
        cost = max(0, length - soft_distance_limit(instance))
        push_column(master, Column(cost, [root(instance), i, root(instance)], BitSet(i)))
    end
end

function add_pairs(master, instance)
    for i in 2:n_nodes(instance)
        for j in 2:n_nodes(instance)
            if i == j
                continue
            end

            if request_quantity(instance, i) + request_quantity(instance, j) > vehicle_capacity(instance)
                continue
            end

            rtime = max(traveltime(instance, root(instance), i), request_twstart(instance, i))
            rtime += traveltime(instance, i, j)

            if rtime > request_twend(instance, j)
                continue
            end

            rtime = max(rtime, request_twstart(instance, j))

            rtime += traveltime(instance, j, root(instance))

            if rtime > request_twend(instance, root(instance))
                continue
            end

            if rtime > vehicle_max_travel_time(instance)
                continue
            end

            length = distance(instance, root(instance), i) + distance(instance, i, j) + distance(instance, j, root(instance))
            if length > hard_distance_limit(instance)
                continue
            end

            # cost = distance(instance, root(instance), i) + distance(instance, i, j) + distance(instance, j, root(instance))
            cost = max(0, length - soft_distance_limit(instance))

            route = [root(instance), i, j, root(instance)]

            nodes = BitSet()
            push!(nodes, i)
            push!(nodes, j)

            push_column(master, Column(cost, route, nodes))
        end
    end
end

function isinteger(master)
    # TODO: Ajouter
    for x in master.ngx
        v = value(x)
        if v > myeps
            return false
        end
    end

    for x in master.x
        v = value(x)
        if v > myeps && v < 1 - myeps
            return false
        end
    end

    return true
end

function check_branching_0(u, v, route)
    @inbounds for i in 1:length(route)-1
        if route[i] == u && route[i+1] == v
            return false
        end
    end

    return true
end

function check_branching_1(u, v, route)
    @inbounds for i in 1:length(route)-1
        if (route[i] == u && route[i+1] != v) || (route[i] != u && route[i+1] == v)
            return false
        end
    end

    return true
end

function check_branching(decisions, route)
    for (u, v, value) in decisions
        if (value == 0 && !check_branching_0(u, v, route)) || (value == 1 && !check_branching_1(u, v, route))
            return false
        end
    end

    return true
end

function set_branching(x, columns, decisions)
    idx = Set{Int}()

    for i in eachindex(x)
        @inbounds if !check_branching(decisions, columns[i].route)
            @inbounds set_upper_bound(x[i], 0.0)
            push!(idx, i)
        end
    end

    return idx
end

function set_branching(master, decisions)
    idx = set_branching(master.x, master.columns, decisions)
    ngidx = set_branching(master.ngx, master.ngcolumns, decisions)

    return idx, ngidx
end

function set_branching(x::Vector{VariableRef}, columns::Vector{Column}, u::Int, v::Int, flow::Int, idxparent::Set{Int})
    idx = Set{Int}()

    for i in eachindex(x)
        if in(i, idxparent)
            continue
        end

        @inbounds route = columns[i].route

        if (flow == 0 && !check_branching_0(u, v, route)) || (flow == 1 && !check_branching_1(u, v, route))
            @inbounds set_upper_bound(x[i], 0.0)
            push!(idx, i)
        end
    end

    return idx
end

function set_branching(master::Master, u::Int, v::Int, flow::Int, idxparent::Set{Int}, ngidxparent::Set{Int})
    idx = set_branching(master.x, master.columns, u, v, flow, idxparent)
    ngidx = set_branching(master.ngx, master.ngcolumns, u, v, flow, ngidxparent)

    return idx, ngidx
end

function unset_branching(x, idx)
    for i in idx
        @inbounds delete_upper_bound(x[i])
    end
end

function unset_branching(master::Master, idx::Set{Int}, ngidx::Set{Int})
    unset_branching(master.x, idx)
    unset_branching(master.ngx, ngidx)
end


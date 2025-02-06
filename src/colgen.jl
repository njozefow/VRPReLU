function call_heuristic(sp::Subproblem, master::Master)
    routes = heuristic(sp)

    for (_, column) in routes
        push_column(master, column)
    end

    return !isempty(routes)
end

function call_ngdynprog(sp::Subproblem, master::Master, lb, upper_bound=typemax(Int))
    routes = ngdynprog(sp, true)

    for (_, column) in routes
        push_column(master, column)
    end

    return !isempty(routes)
end

function call_ngdynprog_heur(sp::Subproblem, master::Master)
    routes = ngdynprog(sp, false)

    for (_, column) in routes
        push_column(master, column)
    end

    return !isempty(routes)
end

@inline colgen(param, master::Master, sp::Subproblem) = colgen(param, master, sp, Vector{Tuple{Int,Int,Int}}())
function colgen(param::Param, master::Master, sp::Subproblem, branchments::Vector{Tuple{Int,Int,Int}}, upperbound=Inf)
    iteration = 0
    dp_iteration = 0

    status = 0

    pibar = zeros(Float64, n_nodes(sp))

    while true
        iteration += 1

        set_stabilization_obj_coeff(sp.instance, master, pibar)

        optimize(master, remaining_time(param))

        if timeout(param)
            break
        end

        lb = objective_value(master)

        set(sp, master.constraints)
        set_branching(sp, branchments)
        build_adj(sp)
        build_ng(sp)

        # save previous pi
        copy!(pibar, sp.pi)

        if status == 0 && call_heuristic(sp, master)
            continue
        elseif status == 0
            status = 1
        end

        if status == 1 && call_ngdynprog_heur(sp, master)
            continue
        elseif status == 1
            status = 2
        end

        dp_iteration += 1

        if status == 2 && call_ngdynprog(sp, master, lb)
            continue
        end

        break
    end

    remove_stabilization(sp.instance, master)
    optimize(master, remaining_time(param))

    return iteration, dp_iteration
end
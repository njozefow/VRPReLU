function call_heuristic(sp::Subproblem, master::Master)
    routes = heuristic(sp)

    for (_, column) in routes
        push_column(master, column)
    end

    return !isempty(routes)
end

function call_ngdynprog(sp::Subproblem, master::Master, lb, upper_bound=typemax(Int))
    bestrc, routes = ngdynprog(sp)

    for column in routes
        push_column(master, column)
    end

    return !isempty(routes) && lb + bestrc < upper_bound - 1.0 + myeps
end

@inline colgen(param, master::Master, sp::Subproblem) = colgen(param, master, sp, Vector{Tuple{Int,Int,Int}}())
function colgen(param::Param, master::Master, sp::Subproblem, branchments::Vector{Tuple{Int,Int,Int}}, upperbound=Inf)
    while true
        optimize(master, remaining_time(param))

        if timeout(param)
            break
        end

        lb = objective_value(master)

        set(sp, master.constraints)
        set_branching(sp, branchments)
        build_ng(sp)

        if call_heuristic(sp, master)
            continue
        end

        # TODO: a retirer dans un premier temps
        # compute_tbounds(sp)
        # compute_tcb(sp)

        if call_ngdynprog(sp, master, lb)
            continue
        end

        break
    end
end
function call_heuristic(sp::Subproblem, master::Master)
    routes = heuristic(sp)

    for (_, column) in routes
        push_column(master, column)
    end

    return !isempty(routes)
end

function call_ngdynprog(sp::Subproblem, master::Master, lb, upper_bound=typemax(Int))
    bestrc, routes = ngdynprog(sp)

    # println("nb columns = ", length(routes))

    for column in routes
        push_column(master, column)
    end

    return !isempty(routes) && lb + bestrc < upper_bound - 1.0 + myeps
end

@inline colgen(param, master::Master, sp::Subproblem) = colgen(param, master, sp, Vector{Tuple{Int,Int,Int}}())
function colgen(param::Param, master::Master, sp::Subproblem, branchments::Vector{Tuple{Int,Int,Int}}, upperbound=Inf)
    # i = 0

    # prevpi = [0.0 for _ in 1:n_nodes(sp)+1]

    while true
        optimize(master, remaining_time(param))
        # prevpi = dual.(master.constraints)


        if timeout(param)
            break
        end

        lb = objective_value(master)

        # i += 1
        # println("iteration ", i)
        # println("lb = ", lb)
        # println("artificial variable = ", master.artificial_x_present)

        set(sp, master.constraints)
        set_branching(sp, branchments)
        build_ng(sp)

        # TODO: A remettre
        if call_heuristic(sp, master)
            continue
        end

        if call_ngdynprog(sp, master, lb)
            continue
        end

        break
    end
end
function compute_flows(x, columns, flows)
    xstar = [(i, value(x[i])) for i in eachindex(x) if value(x[i]) > myeps]

    for (k, value) in xstar
        @inbounds route = columns[k].route

        @inbounds for i in 2:length(route)-2
            @inbounds u = route[i]
            @inbounds v = route[i+1]

            if haskey(flows, (u, v))
                flows[(u, v)] += value
            else
                flows[(u, v)] = value
            end
        end
    end
end

function select_branching(sp, flows)
    minflow = Inf
    minu = 0
    minv = 0

    for ((u, v), value) in flows
        if value > myeps && value < 1.0 - myeps && abs(value - 0.5) * distance(sp, u, v) < minflow
            minflow = abs(value - 0.5) * distance(sp, u, v)
            minu = u
            minv = v
        end
    end

    return minu, minv
end

function search_branching(master, sp)
    flows = Dict{Tuple{Int,Int},Float64}()

    compute_flows(master.x, master.columns, flows)
    compute_flows(master.ngx, master.ngcolumns, flows)

    return select_branching(sp, flows)
end

function generate_child(param, master, subproblem, upperbound, decisions, idxparent, ngidxparent, u, v, flow)
    # Step 1: update master with new flow
    idx, ngidx = set_branching(master, u, v, flow, idxparent, ngidxparent)

    # Step 2: call the gencol
    push!(decisions, (u, v, flow))
    colgen(param, master, subproblem, decisions, upperbound)
    pop!(decisions)

    # Step 3: get results
    feasible = !master.artificial_x_present
    integer = feasible ? isinteger(master) : false
    lowerbound = feasible ? objective_value(master) : Inf

    # Step 4: reset the master
    unset_branching(master, idx, ngidx)

    # Step 5: return results
    return feasible, integer, lowerbound
end

# function explore_node(param, master, subproblem, nodes, upperbound, decisions)
function explore_node(param, master, subproblem, nodes, upperbound)
    ubtime = Inf
    # lb = peek(nodes)[2]
    decisions = dequeue!(nodes)

    # Step 1: set the branchments
    idx, ngidx = set_branching(master, decisions)

    # Step 2: solve the updated master -> it should be possible to get from the priority queue
    optimize(master)

    # println("lowerbound = ", objective_value(master))

    # Step 4: search new flow to branch on
    u, v = search_branching(master, subproblem)

    if u == 0 && v == 0 # the solution is integer
        ub = objective_value(master)
        if round(ub) < upperbound
            ubtime = cpu_time(param)
            upperbound = round(ub)
        end
        return upperbound, ubtime < Inf, ubtime
    end

    # Step 5: build children fixing the flow to 1
    feasible_one, integer_one, lowerbound_one = generate_child(param, master, subproblem, upperbound, decisions, idx, ngidx, u, v, 1)
    if integer_one && lowerbound_one < upperbound # new upper bound
        # println("new upper bound : ", lowerbound_one)
        upperbound = lowerbound_one
        ubtime = cpu_time(param)
    end

    # Step 6: build children fixing the flow to 0
    feasible_zero, integer_zero, lowerbound_zero = generate_child(param, master, subproblem, upperbound, decisions, idx, ngidx, u, v, 0)
    if integer_zero && lowerbound_zero < upperbound # new upper bound
        # println("new upper bound : ", lowerbound_zero)
        upperbound = lowerbound_zero
        ubtime = cpu_time(param)
    end

    # Step 7: add children to nodes if they are feasible, not integer and not dominaned
    # TODO: Vérifier ici pourquoi je n'avais -1 
    if !integer_one && feasible_one && lowerbound_one < upperbound - 1.0 - myeps
        branching_one = copy(decisions)
        push!(branching_one, (u, v, 1))
        enqueue!(nodes, branching_one, lowerbound_one)
    end

    if !integer_zero && feasible_zero && lowerbound_zero < upperbound - 1.0 - myeps
        branching_zero = copy(decisions)
        push!(branching_zero, (u, v, 0))
        enqueue!(nodes, branching_zero, lowerbound_zero)
    end

    # Step 8: reset the master
    unset_branching(master, idx, ngidx)

    # Step 8: return the upper bound in case it was updated (the master and nodes are updated and passed by reference)
    return upperbound, ubtime < Inf, ubtime
end

function search_tree(param::Param, master::Master, subproblem::Subproblem)
    nodes = PriorityQueue{Vector{Tuple{Int,Int,Int}},Float64}()

    optimize(master, remaining_time(param))
    enqueue!(nodes, Vector{Tuple{Int,Int,Int}}(), objective_value(master))

    upperbound = typemax(Int)
    ubtime = Inf

    while !isempty(nodes) && peek(nodes)[2] < upperbound - 1.0 + myeps && !timeout(param)
        upperbound, updated, cputime = explore_node(param, master, subproblem, nodes, upperbound)
        if updated
            ubtime = cputime
        end
    end

    return upperbound, ubtime, (isempty(nodes) || peek(nodes)[2] >= upperbound - myeps)
end
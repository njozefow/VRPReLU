# function hinit(sp::Subproblem)
function hinit(sp)
    labels = [Label() for _ in 1:n_nodes(sp), _ in 1:vehicle_capacity(sp)]

    lid = 1

    for i in 2:n_nodes(sp)
        set_label(sp, labels[i, request_quantity(sp, i)], i, lid)

        lid += 1
    end

    return lid, labels
end

function hrun(sp, labels, lid)
    for q in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            label = labels[i, q]

            # Pas de label à étendre
            if label.id == -1
                continue
            end

            for (j, dij, pij, tij) in adj(sp, i)
                if !allowed(sp, label, j, dij, tij)
                    continue
                end

                labelto = labels[j, label.load+request_quantity(sp, j)]

                cost = max(0, label.length + dij - soft_distance_limit(sp))
                rc = cost + label.picost + pij

                if rc < path_reduced_cost(sp, labelto) - myeps
                    set_extend(sp, label, labelto, j, lid)
                    lid += 1
                end
            end
        end
    end
end

function hbuildroute(sp, labels, i, d)
    route = Vector{Int}()
    push!(route, root(sp))

    label = labels[i, d]
    push!(route, label.node)

    nodes = BitSet(label.node)

    pi = label.pnode
    pd = label.pload

    while pi != root(sp)
        push!(route, pi)
        push!(nodes, pi)
        l = labels[pi, pd]
        pi = l.pnode
        pd = l.pload
    end

    push!(route, root(sp))
    reverse!(route)

    cost = route_cost(sp, label)

    return Column(cost, route, nodes)
end

function hbuildroutes(sp, labels)
    routes = Vector{Tuple{Float64,Column}}()

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            if labels[i, d].id == -1
                continue
            end

            @inbounds rc = route_reduced_cost(sp, labels[i, d])

            if rc > -myeps
                continue
            end

            route = hbuildroute(sp, labels, i, d)

            if length(routes) < max_heuristic_routes
                push!(routes, (rc, route))
            else
                sort!(routes, by=x -> x[1])
                if rc < routes[end][1]
                    pop!(routes)
                    push!(routes, (rc, route))
                end
            end
        end
    end

    return routes
end

function heuristic(sp)
    lid, labels = hinit(sp)

    hrun(sp, labels, lid)

    return hbuildroutes(sp, labels)
end
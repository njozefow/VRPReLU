# function hinit(sp::Subproblem)
function hinit(sp)
    labels = [Label() for _ in 1:n_nodes(sp), _ in 1:vehicle_capacity(sp)]

    lid = 1

    for i in 2:n_nodes(sp)
        set(sp, labels[i, request_quantity(sp, i)], i, lid)

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

            # for j in 2:n_nodes(sp)
            #     if !allowed(label, j, sp)
            #         continue
            #     end
            for (j, dij, pij, tij) in adj(sp, i)
                if !allowed(label, j, dij, tij, sp)
                    continue
                end

                labelto = labels[j, label.load+request_quantity(sp, j)]

                # TODO: Il y a une erreur ici
                # rc = label.reduced_cost + reduced_cost(sp, label.node, j)
                # rc = path_reduced_cost(label)
                cost = max(0, label.distance + dij)
                rc = cost + label.picost + pij

                if rc < path_reduced_cost(labelto) - myeps
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

    if in(-1, route)
        println("erreur")
        println(route)
    end

    # cost = label.distance + distance(sp, label.node, root(sp))
    cost = route_cost(sp, label)

    return Column(cost, route, nodes)
end

function hbuildroutes(sp, labels)
    routes = Vector{Tuple{Float64,Column}}()

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            # rc = labels[i, d].reduced_cost + reduced_cost(sp, i, root(sp))
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
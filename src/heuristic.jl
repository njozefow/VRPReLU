# function hinit(sp::Subproblem)
function hinit(sp)
    # (reduced_cost, u, pnode, pload)
    labels = [Label(sp) for _ in 1:n_nodes(sp), _ in 1:vehicle_capacity(sp)]

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

            for j in 2:n_nodes(sp)
                if !allowed(label, j, sp)
                    continue
                end

                labelto = labels[j, label.load+request_quantity(sp, j)]

                rc = label.reduced_cost + reduced_cost(sp, label.node, j)

                if rc < labelto.reduced_cost - myeps
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

    cost = label.distance + distance(sp, label.node, root(sp))

    push!(route, root(sp))

    reverse!(route)

    return Column(cost, route, nodes)
end

function hbuildroutes(sp, labels)
    routes = Vector{Tuple{Float64,Column}}()

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            rc = labels[i, d].reduced_cost + reduced_cost(sp, i, root(sp))

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
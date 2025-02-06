function ngdpinit(sp)
    lid = 1

    buckets = [Vector{Label}() for _ in 1:n_nodes(sp), _ in 1:vehicle_capacity(sp)]

    for i in 2:n_nodes(sp)
        label = Label(sp, i, lid)
        @inbounds push!(buckets[i, request_quantity(sp, i)], label)
        lid += 1
    end

    return lid, buckets
end

function ngdprun(sp, buckets, lid, dom)
    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            if isempty(buckets[i, d])
                continue
            end

            for l in buckets[i, d]
                for (j, dij, pij, tij) in adj(sp, i)
                    if !allowed(l, j, dij, tij, sp)
                        continue
                    end

                    newlabel = Label(sp, l, j, dij, pij, tij, lid)

                    ok = true

                    for q in 1:newlabel.load-1
                        @inbounds bucket = buckets[j, q]

                        if isempty(bucket)
                            continue
                        end

                        if dominate(bucket, newlabel, dom)
                            ok = false
                            break
                        end
                    end

                    @inbounds if ok && push(buckets[j, newlabel.load], newlabel, dom)
                        for q in newlabel.load+1:vehicle_capacity(sp)

                            @inbounds bucket = buckets[j, q]

                            if isempty(bucket)
                                continue
                            end

                            clean(bucket, newlabel, dom)
                        end

                        lid += 1
                    end
                end
            end
        end
    end
end

function search_bucket(bucket, pid)
    for l in bucket
        if l.id == pid
            return l
        end
    end
end

function ngbuildroute(sp, buckets, label)
    route = Vector{Int}()
    push!(route, root(sp))
    push!(route, label.node)

    nodes = BitSet(label.node)

    # cost = label.distance + distance(sp, label.node, root(sp))
    cost = route_cost(sp, label)

    pi = label.pnode
    pd = label.pload
    pid = label.pid

    while pi != root(sp)
        @inbounds labels = buckets[pi, pd]

        l = search_bucket(labels, pid)

        push!(route, pi)
        push!(nodes, pi)

        pi = l.pnode
        pd = l.pload
        pid = l.pid
    end

    push!(route, root(sp))

    reverse!(route)

    return Column(cost, route, nodes)
end

function insert(routes, rc, route)
    push!(routes, (rc, route))
    pos = length(routes)
    while pos > 2 && routes[pos][1] < routes[pos-1][1]
        routes[pos], routes[pos-1] = routes[pos-1], routes[pos]
        pos -= 1
    end
end

function ngbuildroutes(sp, buckets, maxroutes, bucket, routes)
    for l in bucket
        rc = route_reduced_cost(sp, l)

        # consider only negative reduced costs
        if rc > -myeps
            continue
        end

        # consider only the maxroutes best routes
        if length(routes) == maxroutes && rc > routes[end][1] - myeps
            continue
        end

        route = ngbuildroute(sp, buckets, l)

        if length(routes) == maxroutes
            pop!(routes)
        end

        # push!(routes, (rc, route))
        # sort!(routes, by=x -> x[1])
        insert(routes, rc, route)
    end
end

function ngbuildroutes(sp, buckets, maxroutes)
    routes = Vector{Tuple{Float64,Column}}()

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            @inbounds bucket = buckets[i, d]

            if isempty(bucket)
                continue
            end

            ngbuildroutes(sp, buckets, maxroutes, bucket, routes)
        end
    end

    return routes
end

function ngdynprog(sp, full)
    lid, buckets = ngdpinit(sp)

    dom = full ? dominates_full : dominates_ressources

    ngdprun(sp, buckets, lid, dom)

    return ngbuildroutes(sp, buckets, full ? max_exact_routes : max_heuristic_routes)
    # return ngbuildroutes(sp, buckets)
    # return ngbuildroutes(sp, buckets, nrc_routes)
end
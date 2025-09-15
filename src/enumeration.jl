function push(sp, bucket, label::EnumLabel)
    pos = 1

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if l.reduced_cost >= path_reduced_cost(sp, label) - myeps
            break
        elseif dominates(l, label)
            return false
        else
            pos += 1
        end
    end

    # Labels with the same reduced cost -> dominate, dominated, or equal
    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if abs(l.reduced_cost - path_reduced_cost(sp, label)) >= myeps
            break
        elseif equal(label, l) || dominates(l, label)
            return false
        elseif dominates(label, l)
            deleteat!(bucket, pos)
        else
            pos += 1
        end
    end

    # Search for first non-dominated label
    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if !dominates(label, l)
            break
        else
            deleteat!(bucket, pos)
        end
    end

    addpos = pos
    pos += 1

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if dominates(label, l)
            deleteat!(bucket, pos)
        else
            pos += 1
        end
    end

    insert!(bucket, addpos, label)

    return true
end

function enuminit(sp)
    lid = 1

    buckets = [Vector{EnumLabel}() for _ in 1:n_nodes(sp), _ in 1:vehicle_capacity(sp)]

    for i in 2:n_nodes(sp)
        println("i: ", i)

        bound = Inf

        rtime = max(traveltime(sp, root(sp), i) + 1, request_twstart(sp, i))
        dist = distance(sp, root(sp), i)
        pc = picost(sp, root(sp), i)

        # TODO: voir si on peut en retirer ici
        # for ng in 1:128
        #     if sp.tbounds[ng, i, tmax(sp)-rtime] < bound - myeps
        #         bound = sp.tbounds[ng, i, tmax(sp)-rtime]
        #     end
        # end

        bound = search_pcb(sp, UInt8(0), i, tmax(sp) - rtime, dist, pc)
        # Il faut récupérer le coût à partir des archives de Pareto
        # if reduced_cost(sp, root(sp), i) + bound > sp.gap - myeps
        #     continue
        # end

        bound > sp.gap - myeps && continue

        label = EnumLabel(sp, i, lid)
        push!(buckets[i, request_quantity(sp, i)], label)
        lid += 1
    end

    return lid, buckets
end

function enumrun(param, sp, buckets, lid)
    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            # if timeout(param)
            #     return
            # end
            timeout(param) && return

            # @inbounds if isempty(buckets[i, d])
            #     continue
            # end
            isempty(buckets[i, d]) && continue

            @inbounds for l in buckets[i, d]
                # for j in 2:n_nodes(sp)
                for (j, dij, pij, tij) in adj(sp, i)
                    # if timeout(param)
                    #     return
                    # end
                    timeout(param) && return

                    # if !allowed(l, j, tij, sp)
                    #     continue
                    # end
                    allowed(sp, l, j, dij, tij) || continue

                    newlabel = EnumLabel(sp, l, j, dij, pij, tij, lid)

                    # cb = search_tcb(sp, newlabel.u, newlabel.node, tmax(sp) - newlabel.rtime)

                    # if newlabel.reduced_cost + cb > sp.gap - myeps
                    #     continue
                    # end

                    # TODO: completion bound
                    bound = search_pcb(sp, newlabel.u, newlabel.node, tmax(sp) - newlabel.rtime, newlabel.distance, newlabel.picost)

                    bound > sp.gap - myeps && continue

                    @inbounds if push(sp, buckets[j, newlabel.load], newlabel)
                        lid += 1
                    end
                end
            end
        end
    end
end

function enumbuildroute(sp, buckets, label)
    route = Vector{Int}()
    push!(route, root(sp))
    push!(route, label.node)

    nodes = BitSet(label.node)

    # TODO: ce n'est pas cela le coût -> ajouter soft limits
    cost = label.distance + distance(sp, label.node, root(sp))
    cost = max(0, cost - soft_distance_limit(sp))

    pi = label.pnode
    pd = label.pload
    pid = label.pid

    while pi != root(sp)
        l = search_bucket(buckets[pi, pd], pid)

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

function enumbuildroutes(sp, buckets, node, load, routes)
    for label in buckets[node, load]
        # TODO: il faut recalculer le cout reduit
        # rc = label.reduced_cost + reduced_cost(sp, label.node, root(sp))
        distance = label.distance + distance(sp, label.node, root(sp))
        distance = max(0, distance - soft_distance_limit(sp))
        picost = label.picost + picost(sp, label.node, root(sp))

        distance + picost > sp.gap - myeps && continue

        # if rc > sp.gap - myeps
        #     continue
        # end

        route = enumbuildroute(sp, buckets, label)

        push!(routes, route)
    end
end

function enumbuildroutes(sp, buckets)
    routes = Vector{Column}()

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            if isempty(buckets[i, d])
                continue
            end

            enumbuildroutes(sp, buckets, i, d, routes)
        end
    end

    return routes
end

function enumeration(param, sp)
    lid, buckets = enuminit(sp)

    # TODO: A retirer
    return Vector{Column}()

    if timeout(param)
        return Vector{Column}()
    end

    enumrun(param, sp, buckets, lid)

    if timeout(param)
        return Vector{Column}()
    end

    return enumbuildroutes(sp, buckets)
end

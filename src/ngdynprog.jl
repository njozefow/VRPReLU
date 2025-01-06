# function ngdpinit(sp::Subproblem)
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


# function push_bucket(bucket, label::Label)
#     i = 1
#     sure = false

#     while i <= length(bucket)
#         @inbounds b = bucket[i]

#         if !sure && dominates(b, label)
#             return false
#         elseif dominates(label, b)
#             sure = true
#             @inbounds bucket[i] = bucket[end]
#             pop!(bucket)
#         else
#             i += 1
#         end
#     end

#     push!(bucket, label)

#     return true
# end

# function dominate_bucket(bucket, label)
#     for b in bucket
#         if dominates(b, label)
#             return true
#         end
#     end

#     return false
# end

# function clean_bucket(bucket, label)
#     pos = 1

#     while pos <= length(bucket)
#         @inbounds blabel = bucket[pos]

#         if dominates(label, blabel)
#             @inbounds bucket[pos] = bucket[end]
#             pop!(bucket)
#         else
#             pos += 1
#         end
#     end
# end

function ngdprun(sp, buckets, lid)
    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            if isempty(buckets[i, d])
                continue
            end

            for l in buckets[i, d]
                # for j in 2:n_nodes(sp)
                for (j, dij, pij, tij) in adj(sp, i)
                    # if !allowed(l, j, sp)
                    #     continue
                    # end
                    if !allowed(l, j, dij, tij, sp)
                        continue
                    end

                    # newlabel = Label(sp, l, j, lid)
                    newlabel = Label(sp, l, j, dij, pij, tij, lid)

                    ok = true

                    for q in 1:newlabel.load-1
                        @inbounds bucket = buckets[j, q]

                        if isempty(bucket)
                            continue
                        end

                        if dominate(bucket, newlabel)
                            ok = false
                            break
                        end
                    end

                    @inbounds if ok && push(buckets[j, newlabel.load], newlabel)
                        lid += 1

                        #TODO: nettoyer les buckets plus grands
                        for q in newlabel.load+1:vehicle_capacity(sp)

                            @inbounds bucket = buckets[j, q]

                            if isempty(bucket)
                                continue
                            end

                            clean(bucket, newlabel)
                        end
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

function ngbuildroutes(sp, buckets, bucket, routes)
    bestrc = Inf

    for label in bucket
        # rc = label.reduced_cost + reduced_cost(sp, label.node, root(sp))
        rc = route_reduced_cost(sp, label)

        if rc > -myeps
            continue
        end

        if rc < bestrc - myeps
            bestrc = rc
        end

        route = ngbuildroute(sp, buckets, label)

        push!(routes, route)
    end

    return bestrc
end

function ngbuildroutes(sp, buckets)
    routes = Vector{Column}()
    bestrc = Inf

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            @inbounds bucket = buckets[i, d]

            if isempty(bucket)
                continue
            end

            rc = ngbuildroutes(sp, buckets, bucket, routes)

            if rc < bestrc - myeps
                bestrc = rc
            end
        end
    end

    return bestrc, routes
end

function ngdynprog(sp)
    lid, buckets = ngdpinit(sp)

    ngdprun(sp, buckets, lid)

    return ngbuildroutes(sp, buckets)
end
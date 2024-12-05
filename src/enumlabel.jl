mutable struct EnumLabel
    id::Int

    pid::Int
    pnode::Int
    pload::Int

    node::Int

    reduced_cost::Float64

    distance::Int
    rtime::Int
    load::Int

    u::BitSet
end

@inline EnumLabel() = EnumLabel(-1, 0, 1, 0, -1, Inf, 0, 0, 0, BitSet())

function EnumLabel(sp, node, id)
    redcost = reduced_cost(sp, root(sp), node)

    dist = distance(sp, root(sp), node)
    rtime = max(traveltime(sp, root(sp), node) + tmin(sp), request_twstart(sp, node))
    load = request_quantity(sp, node)

    return EnumLabel(id, 0, root(sp), 0, node, redcost, dist, rtime, load, BitSet(node))
end

function EnumLabel(sp, lfrom, node, lid)
    lto = EnumLabel()

    lto.id = lid

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.node = node

    lto.reduced_cost = lfrom.reduced_cost + reduced_cost(sp, lfrom.node, node)

    lto.distance = lfrom.distance + distance(sp, lfrom.node, node)
    lto.rtime = max(lfrom.rtime + traveltime(sp, lfrom.node, node), request_twstart(sp, node))
    lto.load = lfrom.load + request_quantity(sp, node)

    lto.u = copy(lfrom.u)
    push!(lto.u, node)

    return lto
end

function allowed(label::EnumLabel, node, sp)
    if label.node == node
        return false
    end

    load = label.load + request_quantity(sp, node)
    if load > vehicle_capacity(sp)
        return false
    end

    rtime = label.rtime + traveltime(sp, label.node, node)

    if rtime > request_twend(sp, node)
        return false
    end

    rtime = max(rtime, request_twstart(sp, node))

    if rtime + traveltime(sp, node, root(sp)) > tmax(sp)
        return false
    end

    if in(node, label.u)
        return false
    end

    # TODO: faut-il le mettre ici pour ne pas générer le label inutilement ?
    # Est-ce que l'on ne dépasse pas le gap
    # bound = search_bound(sp, node, load, label.u)
    # if label.reduced_cost + reduced_cost(sp, label.node, node) + bound > sp.gap - myeps
    #     return false
    # end

    return true
end

function dominate_distance(lone::EnumLabel, ltwo::EnumLabel)
    if !issetequal(lone.u, ltwo.u)
        return false
    end

    if lone.rtime != ltwo.rtime
        return false
    end

    if abs(lone.reduced_cost - ltwo.reduced_cost) > myeps
        return false
    end

    return lone.distance < ltwo.distance
end

function dominates(lone::EnumLabel, ltwo::EnumLabel)
    if lone.load != ltwo.load
        return false
    end

    if !issetequal(lone.u, ltwo.u)
        return false
    end

    if lone.rtime > ltwo.rtime
        return false
    end

    # TODO: faut-il le remettre ou si rtime == ltwo.rtime, on différencie sur la distance ?
    # if lone.distance > ltwo.distance
    #     return false
    # end

    if lone.reduced_cost > ltwo.reduced_cost + myeps
        return false
    end

    return true
end
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

    u = BitSet(node)

    return EnumLabel(id, 0, root(sp), 0, node, redcost, dist, rtime, load, u)
end

function EnumLabel(sp, lfrom::EnumLabel, j, dij, rij, tij, lid)
    lto = EnumLabel()

    lto.id = lid

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.node = j

    lto.reduced_cost = lfrom.reduced_cost + rij

    lto.distance = lfrom.distance + dij
    lto.rtime = max(lfrom.rtime + tij, request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    lto.u = copy(lfrom.u)
    push!(lto.u, j)

    return lto
end

function allowed(label::EnumLabel, j, tij, sp)
    if label.node == j
        return false
    end

    if label.load + request_quantity(sp, j) > vehicle_capacity(sp)
        return false
    end

    if label.rtime + tij > request_twend(sp, j)
        return false
    end

    if in(j, label.u)
        return false
    end

    return true
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

    return true
end

function dominates(lone::EnumLabel, ltwo::EnumLabel)
    if lone.rtime > ltwo.rtime
        return false
    end

    if !issetequal(lone.u, ltwo.u)
        return false
    end

    return true
end
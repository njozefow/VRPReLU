mutable struct EnumLabel
    id::Int

    pid::Int
    pnode::Int
    pload::Int

    node::Int

    distance::Int
    rtime::Int
    load::Int

    picost::Float64
    u::BitSet
end

@inline EnumLabel() = EnumLabel(-1, 0, 1, 0, -1, 0, 0, 0, 0.0, BitSet())

function EnumLabel(sp, node, id)
    dist = distance(sp, root(sp), node)
    rtime = max(traveltime(sp, root(sp), node) + tmin(sp), request_twstart(sp, node))
    load = request_quantity(sp, node)

    picost = picost(sp, root(sp), node)

    u = BitSet(node)

    return EnumLabel(id, 0, root(sp), 0, node, dist, rtime, load, picost, u)
end

function EnumLabel(sp, lfrom::EnumLabel, j, dij, pij, tij, lid)
    lto = EnumLabel()

    lto.id = lid

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.node = j

    lto.distance = lfrom.distance + dij
    lto.rtime = max(lfrom.rtime + tij, request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    lto.picost = lfrom.picost + pij

    lto.u = copy(lfrom.u)
    push!(lto.u, j)

    return lto
end

# TODO: il faut prendre en compte la hard limit sur la distance
function allowed(label::EnumLabel, j, dij, tij, sp)
    label.node == j && return false

    label.load + request_quantity(sp, j) > vehicle_capacity(sp) && return false

    path_length(sp, label) + dij + distance(sp, j, root(sp)) > hard_distance_limit(sp) && return false

    # if label.rtime + tij > request_twend(sp, j)
    #     return false
    # end
    label.rtime + tij > request_twend(sp, j) && return false
    max(label.rtime + tij, request_twstart(sp, j)) + traveltime(sp, j, root(sp)) > tmax(sp) && return false

    in(j, label.u) && return false

    return true
end

function allowed(label::EnumLabel, node, sp)
    label.node == node && return false
    # if label.node == node
    #     return false
    # end

    load = label.load + request_quantity(sp, node)
    load > vehicle_capacity(sp) && return false
    # if load > vehicle_capacity(sp)
    #     return false
    # end

    rtime = label.rtime + traveltime(sp, label.node, node)

    rtime > request_twend(sp, node) && return false
    # if rtime > request_twend(sp, node)
    #     return false
    # end

    rtime = max(rtime, request_twstart(sp, node))

    rtime + traveltime(sp, node, root(sp)) > tmax(sp) && return false
    # if rtime + traveltime(sp, node, root(sp)) > tmax(sp)
    #     return false
    # end

    in(node, label.u) && return false
    # if in(node, label.u)
    #     return false
    # end

    return true
end

# TODO: Is it OK to not test distance ?
function dominates(lone::EnumLabel, ltwo::EnumLabel)
    lone.rtime > ltwo.rtime && return false
    # if lone.rtime > ltwo.rtime
    #     return false
    # end

    # TODO: Est-ce correct de faire ce test ?
    lone.distance > ltwo.distance && return false

    !issetequal(lone.u, ltwo.u) && return false
    # if !issetequal(lone.u, ltwo.u)
    #     return false
    # end

    return true
end
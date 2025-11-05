@kwdef mutable struct EnumLabel
    id::Int = -1
    node::Int = 0

    pid::Int = 0
    pnode::Int = 1
    pload::Int = 0

    length::Int = 0
    picost::Float64 = 0.0

    rtime::Int = 0
    load::Int = 0

    u::BitSet = BitSet()
end

# @inline EnumLabel() = EnumLabel(-1, 0, 1, 0, -1, 0, 0, 0, 0.0, BitSet())
@inline path_cost(sp, label::EnumLabel) = max(0, label.length - soft_distance_limit(sp))
@inline path_reduced_cost(sp, label::EnumLabel) = path_cost(sp, label) + label.picost

@inline route_cost(sp, label::EnumLabel) = max(0, label.length + distance(sp, label.node, root(sp)) - soft_distance_limit(sp))
@inline route_reduced_cost(sp, label::EnumLabel) = route_cost(sp, label) + label.picost + picost(sp, label.node, root(sp))

@inline route_length(sp, label::EnumLabel) = label.length + distance(sp, label.node, root(sp))

function EnumLabel(sp, node, id)
    label = EnumLabel()

    label.id = id
    label.node = node

    label.pid = 0
    label.pnode = root(sp)
    label.pload = 0

    label.length = distance(sp, root(sp), node)
    label.picost = picost(sp, root(sp), node)

    label.rtime = max(traveltime(sp, root(sp), node) + 1, request_twstart(sp, node))
    label.load = request_quantity(sp, node)

    label.u = BitSet(node)

    return label
end

function EnumLabel(sp, lfrom::EnumLabel, j, dij, pij, tij, lid)
    lto = EnumLabel()

    lto.id = lid
    lto.node = j

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.length = lfrom.length + dij
    lto.picost = lfrom.picost + pij

    lto.rtime = max(lfrom.rtime + tij, request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    lto.u = copy(lfrom.u)
    push!(lto.u, j)

    return lto
end

function allowed(sp, label::EnumLabel, j, dij, tij)
    label.node == j && return false
    label.load + request_quantity(sp, j) > vehicle_capacity(sp) && return false
    label.length + dij + distance(sp, j, root(sp)) > hard_distance_limit(sp) && return false
    label.rtime + tij > request_twend(sp, j) && return false
    return !in(j, label.u)
end

function equal(lone::EnumLabel, ltwo::EnumLabel)
    lone.rtime != ltwo.rtime && return false
    lone.length != ltwo.length && return false
    return issetequal(lone.u, ltwo.u)
end

# TODO: Is there a problem here ?
function dominates(lone::EnumLabel, ltwo::EnumLabel)
    lone.rtime > ltwo.rtime && return false
    lone.length > ltwo.length && return false
    return issetequal(lone.u, ltwo.u)
end

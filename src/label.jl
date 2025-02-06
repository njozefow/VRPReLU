@kwdef mutable struct Label
    id::Int = -1

    pid::Int = 0  # parent id
    pnode::Int = 1 # parent node
    pload::Int = 0 # parent load

    node::Int = -1

    picost::Float64 = 0.0
    # TODO: ici ce n'est pas bon
    distance::Int = 0 # Initialiser avec soft_distance_limit ?

    rtime::Int = 0 # ready time 
    load::Int = 0

    u::UInt8 = UInt8(0)
end

Label(sp) = Label(distance=-soft_distance_limit(sp))

function Label(sp, node, id)
    label = Label()
    set(sp, label, node, id)
    return label
end

function set(sp, label, node, id)
    label.id = id

    label.pid = 0
    label.pnode = root(sp)
    label.pload = 0

    label.node = node

    label.picost = picost(sp, root(sp), node)

    label.distance = distance(sp, root(sp), node) - soft_distance_limit(sp)
    label.rtime = max(traveltime(sp, root(sp), node) + 1, request_twstart(sp, node))
    label.load = request_quantity(sp, node)

    label.u = UInt8(0)
end

function Label(sp, lfrom, node, lid)
    label = Label(sp)
    set_extend(sp, lfrom, label, node, lid)
    return label
end

@inline path_cost(label) = max(0, label.distance)
@inline path_reduced_cost(label) = path_cost(label) + label.picost

@inline route_cost(sp, label) = max(0, label.distance + distance(sp, label.node, root(sp)))
@inline route_reduced_cost(sp, label) = route_cost(sp, label) + label.picost + picost(sp, label.node, root(sp))

@inline path_length(sp, label) = label.distance + soft_distance_limit(sp)
@inline route_length(sp, label) = label.distance + distance(sp, label.node, root(sp)) + soft_distance_limit(sp)

function Label(sp, lfrom::Label, j, dij, pij, tij, lid)
    lto = Label(sp)

    lto.id = lid

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.node = j

    lto.picost = lfrom.picost + pij
    lto.distance = lfrom.distance + dij
    lto.rtime = max(lfrom.rtime + tij, request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    lto.u = ngintersect(sp.ng, lfrom.node, lfrom.u, j)

    return lto
end

function set_extend(sp, lfrom, lto, node, lid)
    lto.id = lid

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.node = node

    lto.picost = lfrom.picost + picost(sp, lfrom.node, lto.node)

    lto.distance = lfrom.distance + distance(sp, lfrom.node, lto.node)
    lto.rtime = max(lfrom.rtime + traveltime(sp, lfrom.node, lto.node), request_twstart(sp, lto.node))
    lto.load = lfrom.load + request_quantity(sp, lto.node)

    lto.u = ngintersect(sp.ng, lfrom.node, lfrom.u, lto.node)
end

# @inline same_node(n1, n2) = n1 == n2
# @inline enough_capacity(load, quantity, capacity) = load + quantity <= capacity

function allowed(label::Label, j, dij, tij, sp)
    if label.node == j
        return false
    end

    if label.load + request_quantity(sp, j) > vehicle_capacity(sp)
        return false
    end

    if path_length(sp, label) + dij > hard_distance_limit(sp)
        return false
    end

    if path_length(sp, label) + dij + distance(sp, j, root(sp)) > hard_distance_limit(sp)
        return false
    end

    rtime = label.rtime + tij

    if rtime > request_twend(sp, j)
        return false
    end

    if ngin(sp.ng, label.node, label.u, j)
        return false
    end

    return true
end

# TODO: Vérifier notamment si la dominance de Pareto suffit
function dominates_full(lone, ltwo)
    if lone.rtime > ltwo.rtime
        return false
    end

    if lone.distance > ltwo.distance + myeps
        return false
    end

    if lone.u & ltwo.u != lone.u
        return false
    end

    return true
end

function dominates_ressources(lone, ltwo)
    if lone.rtime > ltwo.rtime
        return false
    end

    if lone.distance > ltwo.distance + myeps
        return false
    end

    return true
end
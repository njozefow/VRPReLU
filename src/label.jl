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

# @inline Label(sp) = Label(-1, 0, 1, 0, -1, Inf, 0, 0, 0, UInt8(0))

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

# TODO: il faut revoir la formule ici
@inline route_cost(sp, label) = max(0, label.distance + distance(sp, label.node, root(sp)))
@inline route_reduced_cost(sp, label) = route_cost(sp, label) + label.picost + picost(sp, label.node, root(sp))

@inline path_length(sp, label) = label.distance + soft_distance_limit(sp)
@inline route_length(sp, label) = label.distance + distance(sp, label.node, root(sp)) + soft_distance_limit(sp)

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

@inline same_node(n1, n2) = n1 == n2
@inline enough_capacity(load, quantity, capacity) = load + quantity <= capacity

function allowed(label::Label, node, sp)
    if same_node(label.node, node)
        return false
    end

    if !adjacent(sp, label.node, node)
        return false
    end

    if !enough_capacity(label.load, request_quantity(sp, node), vehicle_capacity(sp))
        return false
    end

    if path_length(sp, label) + distance(sp, label.node, node) > hard_distance_limit(sp)
        return false
    end

    if path_length(sp, label) + distance(sp, label.node, node) + distance(sp, node, root(sp)) > hard_distance_limit(sp)
        return false
    end

    rtime = label.rtime + traveltime(sp, label.node, node)

    if rtime > request_twend(sp, node)
        return false
    end

    rtime = max(rtime, request_twstart(sp, node))

    if rtime + traveltime(sp, node, root(sp)) > vehicle_max_travel_time(sp)
        return false
    end

    if ngin(sp.ng, label.node, label.u, node)
        return false
    end

    return true
end

# TODO: Vérifier notamment si la dominance de Pareto suffit
function dominates(lone, ltwo)
    if lone.load > ltwo.load
        return false
    end

    if lone.rtime > ltwo.rtime
        return false
    end

    # TODO: dominance sur distance -> seulement si soft limit ?
    if lone.distance > ltwo.distance
        return false
    end

    # TODO: ici ce n'est pas cela
    # if lone.reduced_cost > ltwo.reduced_cost + myeps
    #     return false
    # end
    if path_reduced_cost(lone) > path_reduced_cost(ltwo) + myeps
        return false
    end

    if lone.u & ltwo.u != lone.u
        return false
    end

    return true
end
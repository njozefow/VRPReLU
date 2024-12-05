mutable struct Label
    id::Int

    pid::Int # parent id
    pnode::Int # parent node
    pload::Int # parent load

    node::Int

    reduced_cost::Float64

    distance::Int
    rtime::Int # ready time
    load::Int

    u::UInt8 # ng
end

@inline same_node(n1, n2) = n1 == n2
@inline enough_capacity(load, quantity, capacity) = load + quantity <= capacity

function common_allowed(label, node, sp)
    if same_node(label.node, node)
        return false
    end

    if !adjacent(sp, label.node, node)
        return false
    end

    if !enough_capacity(label.load, request_quantity(sp, node), vehicle_capacity(sp))
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

    return true
end

function allowed(label::Label, node, sp)
    if !common_allowed(label, node, sp)
        return false
    end

    if ngin(sp.ng, label.node, label.u, node)
        return false
    end

    return true
end

@inline Label(sp) = Label(-1, 0, 1, 0, -1, Inf, 0, 0, 0, UInt8(0))

function Label(sp, node, id)
    redcost = reduced_cost(sp, root(sp), node)

    dist = distance(sp, root(sp), node)
    rtime = max(traveltime(sp, root(sp), node) + 1, request_twstart(sp, node))
    load = request_quantity(sp, node)

    return Label(id, 0, root(sp), 0, node, redcost, dist, rtime, load, UInt8(0))
end

function common_set(sp, label, node, id)
    label.id = id

    label.pid = 0
    label.pnode = root(sp)
    label.pload = 0

    label.node = node

    label.reduced_cost = reduced_cost(sp, root(sp), node)

    label.distance = distance(sp, root(sp), node)
    label.rtime = max(traveltime(sp, root(sp), node) + 1, request_twstart(sp, node))
    label.load = request_quantity(sp, node)
end

function set(sp, label, node, id)
    common_set(sp, label, node, id)
    label.u = UInt8(0)
end

function common_set_extend(sp, lfrom, lto, node, lid)
    lto.id = lid

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.node = node

    lto.reduced_cost = lfrom.reduced_cost + reduced_cost(sp, lfrom.node, lto.node)

    lto.distance = lfrom.distance + distance(sp, lfrom.node, lto.node)
    lto.rtime = max(lfrom.rtime + traveltime(sp, lfrom.node, lto.node), request_twstart(sp, lto.node))
    lto.load = lfrom.load + request_quantity(sp, lto.node)
end

function set_extend(sp, lfrom, lto, node, lid)
    common_set_extend(sp, lfrom, lto, node, lid)
    lto.u = ngintersect(sp.ng, lfrom.node, lfrom.u, lto.node)
end


function Label(sp, lfrom, node, lid)
    label = Label(sp)
    set_extend(sp, lfrom, label, node, lid)
    return label
end

function common_dominates(lone, ltwo)
    if lone.load > ltwo.load
        return false
    end

    if lone.rtime > ltwo.rtime
        return false
    end

    if lone.reduced_cost > ltwo.reduced_cost + myeps
        return false
    end

    return true
end

function dominates(lone::Label, ltwo::Label)
    if !common_dominates(lone, ltwo)
        return false
    end

    if lone.u & ltwo.u != lone.u
        return false
    end

    return true
end
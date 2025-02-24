# Optimized Label struct with fields grouped for better cache locality
@kwdef mutable struct Label
    # Identifiers and node info
    id::Int = -1
    pid::Int = 0
    node::Int = -1
    pnode::Int = 1

    # Resources
    distance::Int = 0
    rtime::Int = 0
    load::Int = 0
    pload::Int = 0

    # Cost and feasibility
    picost::Float64 = 0.0
    u::UInt8 = UInt8(0)
end

# Optimized helper functions
@inline is_within_soft_limit(label) = label.distance <= 0

@inline same_distance_category(lone, ltwo) = (lone.distance <= 0) == (ltwo.distance <= 0)

@inline dominates_by_resources(lone, ltwo) = !(lone.rtime > ltwo.rtime || lone.distance > ltwo.distance + myeps)

@inline function update_label_ng(sp, lfrom, lto, j)
    within_soft_from = is_within_soft_limit(lfrom)
    within_soft_to = is_within_soft_limit(lto)

    if within_soft_from && !within_soft_to
        lto.u = ngintersect(sp.softng, lfrom.node, lfrom.u, sp.hardng, j)
    elseif !within_soft_from
        lto.u = ngintersect(sp.hardng, lfrom.node, lfrom.u, j)
    elseif within_soft_to
        lto.u = ngintersect(sp.softng, lfrom.node, lfrom.u, j)
    end
end

@inline function can_extend_to_node(sp, label, j, dij, tij)
    if label.node == j ||
       label.load + request_quantity(sp, j) > vehicle_capacity(sp)
        return false
    end

    if path_length(sp, label) + dij + distance(sp, j, root(sp)) > hard_distance_limit(sp)
        return false
    end

    if label.rtime + tij > request_twend(sp, j)
        return false
    end

    ng = is_within_soft_limit(label) ? sp.softng : sp.hardng
    return !ngin(ng, label.node, label.u, j)
end

# Base constructors
Label(sp) = Label(distance=-soft_distance_limit(sp))

function Label(sp, node, id)
    label = Label()
    set(sp, label, node, id)
    return label
end

# Core label functions
@inline path_cost(label) = max(0, label.distance)
@inline path_reduced_cost(label) = path_cost(label) + label.picost

@inline route_cost(sp, label) = max(0, label.distance + distance(sp, label.node, root(sp)))
@inline route_reduced_cost(sp, label) = route_cost(sp, label) + label.picost + picost(sp, label.node, root(sp))

@inline path_length(sp, label) = label.distance + soft_distance_limit(sp)
@inline route_length(sp, label) = label.distance + distance(sp, label.node, root(sp)) + soft_distance_limit(sp)

# Label initialization
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

# Label extension
function Label(sp, lfrom::Label, j, dij, pij, tij, lid)
    lto = Label(sp)

    # Set basic fields
    lto.id = lid
    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load
    lto.node = j

    # Set resource fields
    lto.picost = lfrom.picost + pij
    lto.distance = lfrom.distance + dij
    lto.rtime = max(lfrom.rtime + tij, request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    # Update ng-routes
    update_label_ng(sp, lfrom, lto, j)

    return lto
end

# Set extension function for updating existing labels
function set_extend(sp, lfrom::Label, lto::Label, j, lid)
    # Set basic fields
    lto.id = lid
    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load
    lto.node = j

    # Set resource fields
    lto.picost = lfrom.picost + picost(sp, lfrom.node, j)
    lto.distance = lfrom.distance + distance(sp, lfrom.node, j)
    lto.rtime = max(lfrom.rtime + traveltime(sp, lfrom.node, j), request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    # Update ng-routes
    update_label_ng(sp, lfrom, lto, j)
end

# Feasibility and dominance checking
@inline allowed(label::Label, j, dij, tij, sp) = can_extend_to_node(sp, label, j, dij, tij)

function dominates_full(sp, lone, ltwo)
    if !dominates_by_resources(lone, ltwo)
        return false
    end

    if same_distance_category(lone, ltwo)
        return (lone.u & ltwo.u == lone.u)
    end

    return ngsubsetequal(sp.softng, lone.u, sp.hardng, ltwo.u, lone.node)
end

@inline dominates_ressources(sp, lone, ltwo) = dominates_by_resources(lone, ltwo)
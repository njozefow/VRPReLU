@kwdef mutable struct Label
    id::Int = -1
    node::Int = 0

    # Identifiers and node info
    pid::Int = 0
    pnode::Int = 1
    pload::Int = 0

    # length and reduced cost
    length::Int = 0
    picost::Float64 = 0.0

    # Resources
    rtime::Int = 0
    load::Int = 0

    # Neighborhood
    u::UInt8 = UInt8(0)
end

# Core label functions
@inline path_cost(sp, label::Label) = max(0, label.length - soft_distance_limit(sp))
@inline path_reduced_cost(sp, label::Label) = path_cost(sp, label) + label.picost

@inline route_cost(sp, label::Label) = max(0, label.length + distance(sp, label.node, root(sp)) - soft_distance_limit(sp))
@inline route_reduced_cost(sp, label::Label) = route_cost(sp, label) + label.picost + picost(sp, label.node, root(sp))

@inline route_length(sp, label::Label) = label.length + distance(sp, label.node, root(sp))

# Base constructors
function Label(sp, node, id)
    label = Label()
    set_label(sp, label, node, id)
    return label
end

# Label initialization
function set_label(sp, label, node, id)
    label.id = id
    label.node = node

    label.pid = 0
    label.pnode = root(sp)
    label.pload = 0

    label.length = distance(sp, root(sp), node)
    label.picost = picost(sp, root(sp), node)

    label.rtime = max(traveltime(sp, root(sp), node) + 1, request_twstart(sp, node))
    label.load = request_quantity(sp, node)

    label.u = UInt8(0)
end

# Label extension
function Label(sp, lfrom::Label, j, dij, pij, tij, lid)
    lto = Label()

    lto.id = lid
    lto.node = j

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.length = lfrom.length + dij
    lto.picost = lfrom.picost + pij

    lto.rtime = max(lfrom.rtime + tij, request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    lto.u = ngintersect(sp.ng, lfrom.node, lfrom.u, j)

    return lto
end

# Set extension function for updating existing labels
function set_extend(sp, lfrom::Label, lto::Label, j, lid)
    lto.id = lid
    lto.node = j

    lto.pid = lfrom.id
    lto.pnode = lfrom.node
    lto.pload = lfrom.load

    lto.length = lfrom.length + distance(sp, lfrom.node, j)
    lto.picost = lfrom.picost + picost(sp, lfrom.node, j)

    lto.rtime = max(lfrom.rtime + traveltime(sp, lfrom.node, j), request_twstart(sp, j))
    lto.load = lfrom.load + request_quantity(sp, j)

    lto.u = ngintersect(sp.ng, lfrom.node, lfrom.u, j)
end

function allowed(sp, label::Label, j, dij, tij)
    label.node == j && return false

    label.load + request_quantity(sp, j) > vehicle_capacity(sp) && return false

    label.rtime + tij > request_twend(sp, j) && return false

    label.length + dij + distance(sp, j, root(sp)) > hard_distance_limit(sp) && return false

    return !ngin(sp.ng, label.node, label.u, j)
end

@inline equal(lone::Label, ltwo::Label) = lone.rtime == ltwo.rtime && lone.length == ltwo.length && lone.u == ltwo.u

@inline dominates_ressources(lone, ltwo) = lone.rtime <= ltwo.rtime && lone.length <= ltwo.length

@inline dominates_full(lone, ltwo) = dominates_ressources(lone, ltwo) && lone.u & ltwo.u == lone.u
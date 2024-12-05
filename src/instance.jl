struct Node
    id::Int
    type::Int
    x::Float64
    y::Float64
end

struct VehicleProfile
    type::Int
    number::Int
    departure_node::Int
    arrival_node::Int
    capacity::Int
    max_travel_time::Int
end

mutable struct Request
    id::Int
    node::Int
    twstart::Int
    twend::Int
    quantity::Int
    service_time::Int
end

# TODO: split pi0 ?
struct Instance
    nodes::Vector{Node}
    fleet::Vector{VehicleProfile}
    requests::Vector{Request}
    distance::Matrix{Int}
    traveltime::Matrix{Int}
    n_vehicles::Int
    soft_distance_limit::Int
    hard_distance_limit::Int
end

@inline n_nodes(instance::Instance) = length(instance.nodes)
@inline root(instance::Instance) = 1
@inline node_id(instance::Instance, i::Int) = @inbounds instance.nodes[i].id
@inline node_type(instance::Instance, i::Int) = @inbounds instance.nodes[i].type
@inline node_x(instance::Instance, i::Int) = @inbounds instance.nodes[i].x
@inline node_y(instance::Instance, i::Int) = @inbounds instance.nodes[i].y

# @inline n_vehicles(instance::Instance) = @inbounds instance.fleet[1].number
@inline n_vehicles(instance::Instance) = instance.n_vehicles
@inline soft_distance_limit(instance::Instance) = @inbounds instance.soft_distance_limit
@inline hard_distance_limit(instance::Instance) = @inbounds instance.hard_distance_limit
@inline vehicle_departure_node(instance::Instance) = @inbounds instance.fleet[1].departure_node
@inline vehicle_arrival_node(instance::Instance) = @inbounds instance.fleet[1].arrival_node
@inline vehicle_capacity(instance::Instance) = @inbounds instance.fleet[1].capacity
@inline vehicle_max_travel_time(instance::Instance) = @inbounds instance.fleet[1].max_travel_time

@inline tmin(instance::Instance) = @inbounds instance.requests[1].twstart
@inline tmax(instance::Instance) = @inbounds instance.requests[1].twend
@inline request_twstart(instance::Instance, i::Int) = @inbounds instance.requests[i].twstart
@inline request_twend(instance::Instance, i::Int) = @inbounds instance.requests[i].twend
@inline request_quantity(instance::Instance, i::Int) = @inbounds instance.requests[i].quantity
@inline request_service_time(instance::Instance, i::Int) = @inbounds instance.requests[i].service_time

@inline distance(instance::Instance, i::Int, j::Int) = @inbounds instance.distance[i, j]
@inline traveltime(instance::Instance, i::Int, j::Int) = @inbounds instance.traveltime[i, j]

@inline backward_distance(instance::Instance, i::Int, j::Int) = @inbounds instance.distance[j, i]
@inline backward_traveltime(instance::Instance, i::Int, j::Int) = @inbounds instance.traveltime[j, i]

function artificial_weight(instance::Instance)
    artificial = 0

    for i in 1:n_nodes(instance)
        artificial += maximum(view(instance.distance, :, i))
    end

    return artificial
end

function read_network(network)
    nodeSet = Vector{Node}()

    nodes = firstelement(network)

    for node in eachelement(nodes)
        id = node["id"]
        type = node["type"]

        coords = firstelement(node)
        x = coords.content

        coords = nextelement(coords)
        y = coords.content

        push!(nodeSet, Node(parse(Int, id), parse(Int, type), parse(Float64, x), parse(Float64, y)))
    end

    return nodeSet
end

function read_fleet(xmlfleet)
    fleet = Vector{VehicleProfile}()

    for vehicle in eachelement(xmlfleet)
        type = vehicle["type"]
        number = vehicle["number"]

        element = firstelement(vehicle)
        departure_node = element.content
        element = nextelement(element)
        arrival_node = element.content
        element = nextelement(element)
        capacity = element.content
        element = nextelement(element)
        max_travel_time = element.content

        push!(fleet, VehicleProfile(parse(Int, type), parse(Int, number), parse(Int, departure_node), parse(Int, arrival_node), parse(Float64, capacity), parse(Float64, max_travel_time)))
    end

    return fleet
end

function read_requests(xmlrequests)
    requests = Vector{Request}()

    push!(requests, Request(-1, 0, 1, 0, 0, 0))

    for request in eachelement(xmlrequests)
        id = request["id"]
        node = request["node"]

        it = firstelement(request)

        start = firstelement(it)
        twstart = start.content
        start = nextelement(start)
        twend = start.content

        it = nextelement(it)
        quantity = it.content

        it = nextelement(it)
        service_time = it.content

        push!(requests, Request(parse(Int, id), parse(Int, node), parse(Int, twstart) + 1, parse(Int, twend) + 1, parse(Float64, quantity), parse(Float64, service_time)))
    end

    return requests
end

function read(filename::String, filerelu, ratio)
    doc = EzXML.readxml(filename)

    root = doc.root

    nodes = Vector{Node}()
    fleet = Vector{VehicleProfile}()
    requests = Vector{Request}()

    for e in eachelement(root)
        if e.name == "network"
            nodes = read_network(e)
        elseif e.name == "fleet"
            fleet = read_fleet(e)
        elseif e.name == "requests"
            requests = read_requests(e)
        end
    end

    requests[1].twstart = 1
    requests[1].twend = fleet[1].max_travel_time + 1

    distance = zeros(Int, Base.length(nodes), Base.length(nodes))

    for i in axes(distance, 1)
        for j in axes(distance, 2)
            distance[i, j] = round(sqrt((nodes[i].x - nodes[j].x)^2 + (nodes[i].y - nodes[j].y)^2))
        end
    end

    traveltime = copy(distance)
    for i in 1:length(nodes)
        for j in 1:i-1
            traveltime[i, j] += requests[i].service_time
        end

        for j in i+1:length(nodes)
            traveltime[i, j] += requests[i].service_time
        end
    end

    io = open(filerelu, "r")
    line = split(readline(io))
    hard_distance_limit = parse(Int, line[1])
    soft_distance_limit = Int(round(hard_distance_limit * ratio))
    n_vehicles = parse(Int, line[2])
    close(io)

    return Instance(nodes, fleet, requests, distance, traveltime, n_vehicles, soft_distance_limit, hard_distance_limit)
end
function ngdpinit(sp)
    lid = 1

    buckets = [Vector{Label}() for _ in 1:n_nodes(sp), _ in 1:vehicle_capacity(sp)]

    for i in 2:n_nodes(sp)
        label = Label(sp, i, lid)
        @inbounds push!(buckets[i, request_quantity(sp, i)], label)
        lid += 1
    end

    return lid, buckets
end

function check_dominance(sp, buckets, j, newlabel, dom)
    for q in 1:newlabel.load-1
        @inbounds bucket = buckets[j, q]
        if !isempty(bucket) && dominate(sp, bucket, newlabel, dom)
            return false
        end
    end
    return true
end

function clean_higher_buckets!(sp, buckets, j, newlabel, max_capacity)
    for q in newlabel.load+1:max_capacity
        @inbounds bucket = buckets[j, q]
        if !isempty(bucket)
            clean(sp, bucket, newlabel, dom)
        end
    end
end

function ngdprun(sp, buckets::Array{Vector{Label},2}, lid::Int, dom)
    capacity = vehicle_capacity(sp)
    nodes = n_nodes(sp)

    for d in 1:capacity
        for i in 2:nodes
            @inbounds bucket = buckets[i, d]
            isempty(bucket) && continue

            for label in bucket
                for (j, dij, pij, tij) in adj(sp, i)
                    # Check if extension is allowed
                    allowed(label, j, dij, tij, sp) || continue

                    # Create new label
                    newlabel = Label(sp, label, j, dij, pij, tij, lid)

                    # Check dominance and add label if not dominated
                    if check_dominance(sp, buckets, j, newlabel, dom) &&
                       @inbounds push(sp, buckets[j, newlabel.load], newlabel, dom)

                        # Clean higher buckets
                        clean_higher_buckets!(sp, buckets, j, newlabel, capacity)
                        lid += 1
                    end
                end
            end
        end
    end

    return lid
end

function find_label_by_id(bucket, parent_id)
    # Search bucket for label with given parent id
    @inbounds for label in bucket
        if label.id == parent_id
            return label
        end
    end
    return nothing # Explicit return for clarity
end

function ngbuildroute(sp, buckets, label)
    route = Vector{Int}()
    push!(route, root(sp))
    push!(route, label.node)

    visited_nodes = BitSet(label.node)
    route_cost_val = route_cost(sp, label)

    current_node = label.pnode
    current_load = label.pload
    current_label_id = label.pid

    while current_node != root(sp)
        @inbounds current_bucket = buckets[current_node, current_load]

        # Find the parent label in the current bucket
        parent_label = find_label_by_id(current_bucket, current_label_id)
        if parent_label === nothing
            error("Invalid route construction: parent label not found")
        end

        push!(route, current_node)
        push!(visited_nodes, current_node)

        # Move to parent node
        current_node = parent_label.pnode
        current_load = parent_label.pload
        current_label_id = parent_label.pid
    end

    push!(route, root(sp))
    reverse!(route)

    return Column(route_cost_val, route, visited_nodes)
end

function insert_sorted_by_cost(routes, reduced_cost, route)
    # Insert new route and sort by reduced cost (ascending)
    push!(routes, (reduced_cost, route))
    sort!(routes, by=first)
end

function ngbuildroutes(sp, buckets, maxroutes, bucket, routes)
    for l in bucket
        rc = route_reduced_cost(sp, l)

        # consider only negative reduced costs
        if rc > -myeps
            continue
        end

        # consider only the maxroutes best routes
        if length(routes) == maxroutes && rc > routes[end][1] - myeps
            continue
        end

        route = ngbuildroute(sp, buckets, l)

        if length(routes) == maxroutes
            pop!(routes)
        end

        insert_sorted_by_cost(routes, rc, route)
    end
end

function ngbuildroutes(sp, buckets, maxroutes)
    routes = Vector{Tuple{Float64,Column}}()

    for d in 1:vehicle_capacity(sp)
        for i in 2:n_nodes(sp)
            @inbounds bucket = buckets[i, d]

            if isempty(bucket)
                continue
            end

            ngbuildroutes(sp, buckets, maxroutes, bucket, routes)
        end
    end

    return routes
end

function ngdynprog(sp, use_exact_dominance::Bool)
    # Initialize labels and buckets
    lid, buckets = ngdpinit(sp)

    # Select dominance function based on the solving mode
    dominance_function = use_exact_dominance ? dominates_full : dominates_ressources

    # Run the main dynamic programming algorithm
    final_lid = ngdprun(sp, buckets, lid, dominance_function)

    # Build routes with appropriate limit based on solving mode
    max_routes = use_exact_dominance ? max_exact_routes : max_heuristic_routes
    return ngbuildroutes(sp, buckets, max_routes)
end
function search_src(instance, master, xstar)
    srcs = PriorityQueue{Tuple{Int,Int,Int},Float64}(Base.Order.Reverse)

    n = n_nodes(instance)
    # constrs = master.constraints

    # Chercher les SRCs non respectées
    for u in 2:n-2, v in u+1:n-1, w in v+1:n
        lhs = 0.0

        # println("u = ", u, " v = ", v, " w = ", w)

        for (i, value) in xstar
            @inbounds nodes = master.columns[i].nodes

            # @inbounds sum = (normalized_coefficient(constrs[u], x) > 0.5) ? 1 : 0
            # @inbounds sum += (normalized_coefficient(constrs[v], x) > 0.5) ? 1 : 0
            # @inbounds sum += (normalized_coefficient(constrs[w], x) > 0.5) ? 1 : 0
            @inbounds sum = in(u, nodes) ? 1 : 0
            @inbounds sum += in(v, nodes) ? 1 : 0
            @inbounds sum += in(w, nodes) ? 1 : 0

            lhs += (sum >= 2) ? value : 0.0
        end

        if lhs > 1.0 + myeps
            enqueue!(srcs, (u, v, w), lhs)
            # println("Nb srcs: ", length(srcs))
        end
    end


    return srcs
    # println("Nb srcs: ", length(srcs))
end

function add_src(cb_data, master, u, v, w, status)
    columns = master.columns

    idx = Vector{Int}()

    for i in eachindex(columns)
        nodes = columns[i].nodes

        cnt = in(u, nodes) ? 1 : 0
        cnt += in(v, nodes) ? 1 : 0
        cnt += in(w, nodes) ? 1 : 0

        if cnt >= 2
            push!(idx, i)
        end
    end

    constr = @build_constraint(sum(master.x[i] for i in idx) <= 1)

    if status == MOI.CALLBACK_NODE_STATUS_FRACTIONAL
        MOI.submit(master.model, MOI.UserCut(cb_data), constr)
    else
        MOI.submit(master.model, MOI.LazyConstraint(cb_data), constr)
    end
end

function bcmaster(instance, master)
    function usercut_callback(cb_data)
        println("Dans usercallback")
        status = callback_node_status(cb_data, master.model)

        if status == MOI.CALLBACK_NODE_STATUS_FRACTIONAL
            xstar = [(i, callback_value(cb_data, master.x[i])) for i in eachindex(master.x) if callback_value(cb_data, master.x[i]) > myeps]
            srcs = search_src(instance, master, xstar)
            println("Nb srcs: ", length(srcs))
            for ((u, v, w), value) in srcs
                println("SRC: ", u, " ", v, " ", w)
                add_src(cb_data, master, u, v, w, status)
            end
        elseif status == MOI.CALLBACK_NODE_STATUS_INTEGER
            println("Dans lazy constraint")
            xstar = [(i, callback_value(cb_data, master.x[i])) for i in eachindex(master.x) if callback_value(cb_data, master.x[i]) > myeps]
            srcs = search_src(instance, master, xstar)
            for ((u, v, w), value) in srcs
                println("SRC: ", u, " ", v, " ", w)
                add_src(cb_data, master, u, v, w, status)
            end
        else
            return
        end
    end

    # unset_silent(master.model)

    set_attribute(master.model, MOI.UserCutCallback(), usercut_callback)
    set_attribute(master.model, MOI.LazyConstraintCallback(), usercut_callback)

    optimize!(master.model)
end


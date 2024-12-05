function init_tbounds(sp)
    sp.tbounds .= Inf

    for i in 2:n_nodes(sp)
        # TODO: faut-il elaguer dès ici ?
        # eta = max(backward_traveltime(sp, root(sp), i) + 1, backward_twstart(sp, i))
        # for t in eta:backward_twend(sp, i), ng in 1:128
        #     sp.tbounds[ng, i, t] = backward_reduced_cost(sp, root(sp), i)
        # end

        for t in backward_twstart(sp, i):backward_twend(sp, i), ng in 1:128
            sp.tbounds[ng, i, t] = backward_reduced_cost(sp, root(sp), i)
        end
    end
end

function run_compute_tbounds(sp)
    for t in 1:tmax(sp), i in 2:n_nodes(sp)
        if t < backward_twstart(sp, i) || t > backward_twend(sp, i)
            continue
        end

        for ng in 1:128
            @inbounds if sp.tbounds[ng, i, t] == Inf
                continue
            end

            for j in 2:n_nodes(sp)
                if i == j || !backward_adjacent(sp, i, j)
                    continue
                end

                eta = max(t + backward_traveltime(sp, i, j), backward_twstart(sp, j))

                if eta > backward_twend(sp, j) || eta > tmax(sp)
                    continue
                end

                ngj = ngintersect(sp.ng, i, UInt8(ng - 1), j)
                @inbounds bound = sp.tbounds[ng, i, t] + backward_reduced_cost(sp, i, j)

                @inbounds if bound < sp.tbounds[ngj+1, j, eta]
                    sp.tbounds[ngj+1, j, eta] = bound
                end
            end
        end
    end
end

function propagate_tbounds(sp)
    for i in 2:n_nodes(sp), ng in 1:128
        for t in backward_twstart(sp, i)+1:backward_twend(sp, i)
            @inbounds if sp.tbounds[ng, i, t] > sp.tbounds[ng, i, t-1]
                sp.tbounds[ng, i, t] = sp.tbounds[ng, i, t-1]
            end
        end
    end
end

function compute_tbounds(sp)
    init_tbounds(sp)
    run_compute_tbounds(sp)
    propagate_tbounds(sp)
end

function search_tcb(sp, u, i, t)
    tcb = Inf

    for ng in 0:127
        if !compatible(sp, i, UInt8(ng), u)
            continue
        end

        @inbounds tcb = min(tcb, sp.tbounds[ng+1, i, t])
    end

    return tcb
end
function init_pbounds(sp::Subproblem)
    empty!.(sp.pbounds)

    for i in 2:n_nodes(sp)
        for t in backward_twstart(sp, i):backward_twend(sp, i), ng in 1:128
            add_point!(sp.pbounds[ng, i, t], backward_distance(sp, root(sp), i), backward_picost(sp, root(sp), i))
        end
    end
end

function run_compute_tbounds(sp)
    for t in 1:tmax(sp), i in 2:n_nodes(sp)
        (t < backward_twstart(sp, i) || t > backward_twend(sp, i)) && continue

        for ng in 1:128
            @inbounds archive = sp.pbounds[ng, i, t]

            isempty(archive) && continue

            for j in 2:n_nodes(sp)
                (i == j || !backward_adjacent(sp, i, j)) && continue

                eta = max(t + backward_traveltime(sp, i, j), backward_twstart(sp, j))

                (eta > backward_twend(sp, j) || eta > tmax(sp)) && continue

                ngj = ngintersect(sp.ng, i, UInt8(ng - 1), j)

                # We must extend all the non dominated point in the archive
                for (d, pc) in archive.elements
                    newd = d + backward_distance(sp, i, j)
                    newpc = pc + backward_picost(sp, i, j)

                    @inbounds add_point!(sp.pbounds[ngj+1, j, eta], newd, newpc)
                end
            end
        end
    end
end

function propagate_pbounds(sp)
    for i in 2:n_nodes(sp), ng in 1:128, t in backward_twstart(sp, i)+1:backward_twend(sp, i)
        @inbounds archive = sp.pbounds[ng, i, t]

        @inbounds merge!(sp.pbounds[ng, i, t], sp.pbounds[ng, i, t-1])
    end
end

function compute_pbounds(sp)
    init_pbounds(sp)
    run_compute_tbounds(sp)
    propagate_pbounds(sp)
end

function search_pcb(sp, u::BitSet, i, t, distance::Int, picost::Float64)
    pcb = Inf

    for ng in 0:127
        !compatible(sp, i, UInt8(ng), u) && continue

        @inbounds elements = sp.pbounds[ng+1, i, t].elements

        # Il faut trouver le point qui marche le mieux
        for (d, pc) in elements
            d += distance # Add distance 

            # do we exceed tne hard distance limit?
            d > hard_distance_limit(sp) && continue

            # do we exceed the soft distance limit ? if not reset d to 0 otherwise set d to the exceed
            d = max(0, d - soft_distance_limit(sp))

            lb = d + picost + pc

            # Is it better than the current best?
            if lb < pcb
                pcb = lb
            end
        end

    end

    return pcb
end
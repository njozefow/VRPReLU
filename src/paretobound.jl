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


            end
        end
    end
end

function compute_pbounds(sp)
    init_pbounds(sp)
    run_compute_tbounds(sp)
end
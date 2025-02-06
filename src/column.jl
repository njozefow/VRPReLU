mutable struct Column
    cost::Int
    route::Vector{Int}
    nodes::BitSet
    # elem::Bool
end

function print_column(c::Column)
    Base.print("cost: ", c.cost)
    Base.print(" / route: ")
    Base.println(c.route, " / length: ", length(c.route) - 2)
end

function compute_distance(sp, column)
    dist = 0

    for i in 1:length(column.route)-1
        dist += distance(sp, column.route[i], column.route[i+1])
    end

    return dist
end

function compute_dual(sp, column)
    dualv = 0.0

    for i in 1:length(column.route)-1
        dualv += sp.pi[column.route[i]]
    end

    return dualv
end

check_distance(sp, column) = compute_distance(sp, column) == column.cost

function check_rc(sp, column, gap)
    d = compute_distance(sp, column)
    rc = d - compute_dual(sp, column)

    return rc < gap - myeps
end
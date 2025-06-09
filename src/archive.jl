struct Archive
    elements::Vector{Tuple{Int,Float64}}

    Archive() = new(Vector{Tuple{Int,Float64}}())
end

@inline add_to_archive!(archive::Archive, objone::Int, objtwo::Float64) = Base.push!(archive.elements, (objone, objtwo))

const FLOAT_TOL = 1e-12

@inline approx_leq(a::Float64, b::Float64) = a <= b + FLOAT_TOL
@inline approx_lt(a::Float64, b::Float64) = a < b - FLOAT_TOL

@inline function dominates(a::Tuple{Int,Float64}, b::Tuple{Int,Float64})
    a1_leq_b1 = a[1] <= b[1]
    a1_lt_b1 = a[1] < b[1]
    a2_lt_b2 = approx_lt(a[2], b[2])
    a2_leq_b2 = approx_leq(a[2], b[2])

    return (a1_leq_b1 && a2_lt_b2) || (a1_lt_b1 && a2_leq_b2)
end

function extract_non_dominated(elements::Vector{Tuple{Int,Float64}}, indices::Vector{Int})
    front = Int[]

    # Move to the first non-dominated element
    idx = 1
    @indounds while idx < length(indices) && elements[indices[idx]][1] == elements[indices[idx+1]][1]
        idx += 1
    end

    return front
end

function rank(archive::Archive)
    n = length(archive.elements)

    indices = collect(1:n)
    sort!(indices, by=i -> (archive.elements[i][1], archive.elements[i][2]))

    fronts = Vector{Vector{Int}}()

    while !isempty(indices)
        front = extract_non_dominated(archive.elements, indices)
        push!(fronts, front)
    end

    return fronts
end




struct NgArchive
    elements::Vector{Tuple{Int,Int,Float64}}

    NgArchive() = new(Vector{Tuple{Int,Int,Float64}}())
    NgArchive(n::Int) = new(Vector{Tuple{Int,Int,Float64}}(undef, n))
end

@inline add_to_archive!(archive::NgArchive, id::Int, objone::Int, objtwo::Float64) = Base.push!(archive.elements, (id, objone, objtwo))

# const FLOAT_TOL = 1e-6

# @inline approx_leq(a::Float64, b::Float64) = a <= b + FLOAT_TOL
@inline approx_lt(a::Float64, b::Float64) = a < b - FLOAT_TOL

@inline function dominates(a::Tuple{Int,Int,Float64}, b::Tuple{Int,Int,Float64})
    @inbounds a1_leq_b1 = a[2] <= b[2]
    @inbounds a1_lt_b1 = a[2] < b[2]
    @inbounds a2_lt_b2 = approx_lt(a[3], b[3])
    @inbounds a2_leq_b2 = approx_leq(a[3], b[3])

    return (a1_leq_b1 && a2_lt_b2) || (a1_lt_b1 && a2_leq_b2)
end

function next_non_weakly_dominated(elements::Vector{Tuple{Int,Int,Float64}}, indices::Vector{Int}, i::Int)
    # Ensure we have a valid starting position
    i >= length(indices) && return i

    # Continue while we have a next element to compare with
    while i < length(indices)  # This ensures indices[i+1] exists
        @inbounds elem_i = elements[indices[i]]
        @inbounds elem_i_next = elements[indices[i+1]]

        if elem_i[2] == elem_i_next[2]
            if abs(elem_i[3] - elem_i_next[3]) < FLOAT_TOL
                deleteat!(indices, i + 1)
                if i >= length(indices)
                    break
                end
            else
                i += 1
            end
        else # Last element in the archive with the same objective 1 value
            break
        end
    end

    return i
end

function extract_non_dominated(elements::Vector{Tuple{Int,Int,Float64}}, indices::Vector{Int})
    front = Int[]

    i = 1 # Start from the first value in indice
    best_picost = Inf

    # i is at the first element to consider
    while i <= length(indices)
        # pass weakly dominated elements and removed same elements
        # in worst case i is equal to length(indices)
        i = next_non_weakly_dominated(elements, indices, i)
        i > length(indices) && break  # If i exceeds indices length, exit loop

        @inbounds idx = indices[i]

        @inbounds current_picost = elements[idx][3]

        if approx_lt(current_picost, best_picost)
            @inbounds push!(front, idx)
            best_picost = current_picost
            deleteat!(indices, i)
        else
            i += 1  # Move to next element since this one wasn't added
        end
    end

    return front
end

function extract_non_dominated_points(archive::NgArchive)
    indices = collect(1:length(archive.elements))
    sort!(indices, by=i -> (archive.elements[i][2], archive.elements[i][3]))

    front = Vector{Tuple{Int,Float64}}()

    i = 1
    best_picost = Inf

    while i <= length(indices)
        i = next_non_weakly_dominated(archive.elements, indices, i)
        i > length(indices) && break  # If i exceeds indices length, exit loop

        @inbounds idx = indices[i]
        @inbounds current_picost = archive.elements[idx][3]

        if approx_lt(current_picost, best_picost)
            @inbounds push!(front, (archive.elements[idx][2], current_picost))
            best_picost = current_picost
            deleteat!(indices, i)
        else
            i += 1  # Move to next element since this one wasn't added
        end
    end

    return front
end

function rank(archive::NgArchive)
    n = length(archive.elements)

    indices = collect(1:n)
    sort!(indices, by=i -> (archive.elements[i][2], archive.elements[i][3]))

    fronts = Vector{Vector{Int}}()

    while !isempty(indices)
        front = extract_non_dominated(archive.elements, indices)
        !isempty(front) && push!(fronts, front)
    end

    return fronts
end

# Select n diversified elements from the archive considering the non-dominated elements in the front
function select_n(front::Vector{Int}, n::Int)
    n >= length(front) && return front
    length(front) == 0 && return Int[]

    if n == 1
        # Select middle element for best representativeness
        mid_idx = (length(front) + 1) ÷ 2
        return [front[mid_idx]]
    elseif n == 2
        # Select extreme points for maximum diversity
        return [front[1], front[end]]
    elseif n == 3
        # Select first, middle, and last
        mid_idx = (length(front) + 1) ÷ 2
        return [front[1], front[mid_idx], front[end]]
    else
        # General case: evenly spaced selection
        step = (length(front) - 1) / (n - 1)
        selected = Int[]

        for i in 0:(n-1)
            idx = round(Int, 1 + i * step)
            push!(selected, front[idx])
        end

        return selected
    end
end

function select_n(archive::NgArchive, n::Int)
    fronts = rank(archive)

    selected = Int[]

    current_rank = 1

    while current_rank <= length(fronts) && n > 0
        @inbounds front = fronts[current_rank]
        front = n >= length(front) ? front : select_n(front, n)

        for idx in front
            @inbounds push!(selected, archive.elements[idx][1])
        end

        n -= length(front)
        current_rank += 1
    end

    return selected
end
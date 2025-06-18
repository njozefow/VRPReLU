struct Archive
    elements::Vector{Tuple{Int,Float64}}

    Archive() = new(Vector{Tuple{Int,Float64}}())
end

@inline add_to_archive!(archive::Archive, objone::Int, objtwo::Float64) = Base.push!(archive.elements, (objone, objtwo))

const FLOAT_TOL = 1e-6

@inline approx_leq(a::Float64, b::Float64) = a <= b + FLOAT_TOL
@inline approx_lt(a::Float64, b::Float64) = a < b - FLOAT_TOL

@inline function dominates(a::Tuple{Int,Float64}, b::Tuple{Int,Float64})
    @inbounds a1_leq_b1 = a[1] <= b[1]
    @inbounds a1_lt_b1 = a[1] < b[1]
    @inbounds a2_lt_b2 = approx_lt(a[2], b[2])
    @inbounds a2_leq_b2 = approx_leq(a[2], b[2])

    return (a1_leq_b1 && a2_lt_b2) || (a1_lt_b1 && a2_leq_b2)
end

function next_non_weakly_dominated(elements::Vector{Tuple{Int,Float64}}, indices::Vector{Int}, i::Int)
    # Ensure we have a valid starting position
    i >= length(indices) && return i

    # Continue while we have a next element to compare with
    while i < length(indices)  # This ensures indices[i+1] exists
        @inbounds elem_i = elements[indices[i]]
        @inbounds elem_i_next = elements[indices[i+1]]

        if elem_i[1] == elem_i_next[1]
            if abs(elem_i[2] - elem_i_next[2]) < FLOAT_TOL
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

function extract_non_dominated(elements::Vector{Tuple{Int,Float64}}, indices::Vector{Int})
    i = 1 # Start from the first value in indice
    best_picost = Inf

    # i is at the first element to consider
    while i <= length(indices)
        # pass weakly dominated elements and removed same elements
        i = next_non_weakly_dominated(elements, indices, i)
        @inbounds current_picost = elements[indices[i]][2]
    end
end

function extract_non_dominated(elements::Vector{Tuple{Int,Float64}}, indices::Vector{Int})
    front = Int[]

    if isempty(indices)
        return front
    end

    # Start from the beginning and find first non-dominated
    i = 1
    i = next_non_dominated(elements, indices, i)

    @inbounds best_picost = elements[indices[i]][2]
    push!(front, indices[i])
    deleteat!(indices, i)

    # Continue from the same position (since we deleted an element, next element shifted down)
    while i <= length(indices)
        i = next_non_dominated(elements, indices, i)
        if i > length(indices)
            break
        end

        @inbounds current_picost = elements[indices[i]][2]

        if current_picost < best_picost - FLOAT_TOL
            best_picost = current_picost
            push!(front, indices[i])
            deleteat!(indices, i)
            # Don't increment i since we deleted an element - next element moved to position i
        else
            i += 1  # Move to next element since this one wasn't added
        end
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




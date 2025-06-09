"""
    NGArchiveElement

A structure to represent elements in the NGArchive with:
- id::Int64: integer identifier for the element
- distance::Int64: integer distance value
- picost::Float64: floating point cost value
"""
struct NGArchiveElement
    id::Int64
    distance::Int64
    picost::Float64
end

"""
    NGArchive

Data structure for storing and managing NGArchiveElements.
"""
mutable struct NGArchive
    elements::Vector{NGArchiveElement}

    # Constructor for empty archive
    NGArchive() = new(Vector{NGArchiveElement}())
end

"""
    add_element!(archive::NGArchive, id::Int64, distance::Int64, picost::Float64)

Add a new element to the NGArchive.
Returns the added element.
"""
function add_element!(archive::NGArchive, id::Int64, distance::Int64, picost::Float64)
    element = NGArchiveElement(id, distance, picost)
    push!(archive.elements, element)
    return element
end

"""
    add_element!(archive::NGArchive, element::NGArchiveElement)

Add an existing NGArchiveElement to the NGArchive.
Returns the added element.
"""
function add_element!(archive::NGArchive, element::NGArchiveElement)
    push!(archive.elements, element)
    return element
end

"""
    is_dominated_by(a::NGArchiveElement, b::NGArchiveElement)

Check if element `a` is dominated by element `b` in a minimization context.
Both distance and picost are to be minimized.

Returns `true` if `b` dominates `a`, `false` otherwise.
"""
function is_dominated_by(a::NGArchiveElement, b::NGArchiveElement)::Bool
    # b dominates a if b is better or equal in all objectives and strictly better in at least one
    return (b.distance <= a.distance && b.picost < a.picost - myeps) ||
           (b.distance < a.distance && b.picost <= a.picost + myeps)
end

function dominates(a::NGArchiveElement, b::NGArchiveElement)::Bool
    return (a.distance <= b.distance && a.picost < b.picost - myeps) ||
           (a.distance < b.distance && a.picost <= b.picost + myeps)
end

"""
    get_nondominated_elements(elements::Vector{NGArchiveElement})

Standalone utility to find all non-dominated elements using an efficient sweep-line algorithm.

!!! note
    This function is now optimized for O(n log n) performance but is no longer used
    by the main `pareto_rank` function. It's kept as a utility for direct use cases
    requiring only the first Pareto front.

# Algorithm
1. Sort elements by distance (ascending)
2. Process elements in order, maintaining the lowest picost seen so far
3. An element is non-dominated if its picost is lower than any previously seen element with less or equal distance

Returns a vector of non-dominated elements.
"""
function get_nondominated_elements(elements::Vector{NGArchiveElement})::Vector{NGArchiveElement}
    # Handle empty or singleton cases
    if length(elements) <= 1
        return copy(elements)
    end

    # Sort elements by distance (ascending)
    sorted_elements = sort(elements, by=e -> e.distance)

    # Result vector for non-dominated elements
    nondominated = Vector{NGArchiveElement}()

    # Add the first element (lowest distance)
    @inbounds push!(nondominated, sorted_elements[1])
    @inbounds last_inserted = nondominated[end]

    for i in 2:length(sorted_elements)
        @inbounds current = sorted_elements[i]

        if current.distance == last_inserted.distance && current.picost < last_inserted.picost - myeps
            @inbounds nondominated[end] = current
            last_inserted = current
            best_picost = current.picost
        elseif current.picost < last_inserted.picost - myeps
            push!(nondominated, current)
            last_inserted = current
        end
    end

    return nondominated
end

"""
    pareto_rank(archive::NGArchive)

Organize elements in the NGArchive by Pareto dominance rank using a fast algorithm.

# Details
- Rank 1: Elements not dominated by any other element (Pareto front)
- Rank 2: Elements not dominated when rank 1 elements are removed
- And so on

Both distance and picost are minimized in the ranking process.

# Algorithm
Uses a fast O(n log n) approach that:
1. Sorts elements by increasing distance
2. Processes elements in a single pass using a modified sweep line algorithm
3. Assigns ranks efficiently without repeated computations

# Returns
A dictionary mapping ranks (starting from 1) to vectors of elements at that rank.
"""
function pareto_rank(archive::NGArchive)::Dict{Int,Vector{NGArchiveElement}}
    # Initialize result dictionary
    ranked_elements = Dict{Int,Vector{NGArchiveElement}}()

    # Handle empty archive case
    if isempty(archive.elements)
        return ranked_elements
    end

    # Sort elements by distance (ascending)
    sorted_elements = sort(copy(archive.elements), by=e -> e.distance)

    # Precompute ranks using the fast non-domination sorting technique
    n = length(sorted_elements)
    ranks = fill(1, n)  # Initialize all elements with rank 1

    # Data structure to track best picosts for each rank
    # best_picosts[r] is the lowest picost seen for rank r
    best_picosts = Dict{Int,Float64}()
    best_picosts[1] = Inf  # Initialize rank 1 with infinity

    # First element is always rank 1
    best_picosts[1] = sorted_elements[1].picost

    # Process remaining elements
    for i in 2:n
        element = sorted_elements[i]

        # Find the appropriate rank for this element
        rank = 1
        while haskey(best_picosts, rank) && element.picost >= best_picosts[rank]
            rank += 1
        end

        # Assign the rank
        ranks[i] = rank

        # Update best picost for this rank
        best_picosts[rank] = min(get(best_picosts, rank, Inf), element.picost)
    end

    # Group elements by their assigned ranks
    for i in 1:n
        rank = ranks[i]
        element = sorted_elements[i]

        if !haskey(ranked_elements, rank)
            ranked_elements[rank] = Vector{NGArchiveElement}()
        end
        push!(ranked_elements[rank], element)
    end

    return ranked_elements
end

"""
    calculate_distances(points)

Calculate pairwise distances between points in normalized objective space.
Returns a distance matrix where distances[i,j] is the distance between points[i] and points[j].
Uses L2-norm (Euclidean distance).

Time complexity: O(n²) where n is the number of points
"""
function calculate_distances(points)
    n = length(points)
    distances = zeros(Float64, n, n)

    for i in 1:n
        for j in i+1:n
            # Euclidean distance
            d = sqrt((points[i][1] - points[j][1])^2 + (points[i][2] - points[j][2])^2)
            distances[i, j] = d
            distances[j, i] = d  # Symmetric
        end
    end

    return distances
end

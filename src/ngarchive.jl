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
    return (b.distance <= a.distance && b.picost < a.picost) ||
           (b.distance < a.distance && b.picost <= a.picost)
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
    push!(nondominated, sorted_elements[1])

    # Current best picost (initialize with first element)
    best_picost = sorted_elements[1].picost

    # Process remaining elements
    for i in 2:length(sorted_elements)
        current = sorted_elements[i]

        # Special case: If current element has same distance as previous element
        if current.distance == sorted_elements[i-1].distance
            # Only keep if it has strictly lower picost
            if current.picost < best_picost
                push!(nondominated, current)
                best_picost = current.picost
            end
            # Normal case: Element has greater distance than previous
        else
            # Only keep if it has strictly lower picost than best seen so far
            if current.picost < best_picost
                push!(nondominated, current)
                best_picost = current.picost
            end
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

"""
    select_diverse_subset(elements::Vector{NGArchiveElement}, k::Int)

Efficiently select k diverse representative elements using a greedy algorithm.
Uses KMeans++ style initialization for selecting diverse points.

Time complexity: O(n·k) where n is number of elements and k is subset size
"""
# function select_diverse_subset(elements::Vector{NGArchiveElement}, k::Int)
#     if length(elements) <= k
#         return copy(elements)
#     end

#     # Extract and normalize objective values (distance and picost)
#     distances = [e.distance for e in elements]
#     picosts = [e.picost for e in elements]

#     # Normalize to [0,1] range
#     dist_min, dist_max = minimum(distances), maximum(distances)
#     picost_min, picost_max = minimum(picosts), maximum(picosts)

#     # Avoid division by zero with small epsilon
#     dist_range = max(dist_max - dist_min, 1.0)
#     picost_range = max(picost_max - picost_min, 1e-6)

#     # Create normalized points in objective space
#     points = [(
#         (distances[i] - dist_min) / dist_range,
#         (picosts[i] - picost_min) / picost_range
#     ) for i in 1:length(elements)]

#     # Algorithm: KMeans++ style initialization (efficient diverse subset selection)
#     # Time complexity: O(n·k)

#     n = length(points)
#     selected = BitSet()  # Keep track of selected indices

#     # Start with the point closest to the ideal point (0,0)
#     dist_to_ideal = [(p[1]^2 + p[2]^2) for p in points]  # No need for sqrt here
#     first_idx = argmin(dist_to_ideal)
#     push!(selected, first_idx)

#     # Select remaining points using KMeans++ style selection
#     # Each time, select the point that maximizes minimum distance to already selected points
#     while length(selected) < k
#         # For each remaining point, calculate minimum squared distance to any selected point
#         min_distances = fill(Inf, n)

#         for i in 1:n
#             if i in selected
#                 continue  # Skip already selected points
#             end

#             # Find minimum distance to any selected point
#             for j in selected
#                 d_squared = (points[i][1] - points[j][1])^2 + (points[i][2] - points[j][2])^2
#                 min_distances[i] = min(min_distances[i], d_squared)
#             end
#         end

#         # Select the point with maximum minimum distance
#         next_point = argmax(min_distances)
#         push!(selected, next_point)
#     end

#     # Return the selected elements
#     return elements[collect(selected)]
# end

"""
    select_representative_elements(archive::NGArchive, n::Int)

Select n representative elements from the archive, prioritizing by Pareto rank.
When a rank has too many elements, select a diverse subset based on objective space distribution.

Time complexity: O(m log m + n·k) where:
- m is total number of elements in archive
- n is number of elements in the largest rank that needs diverse selection
- k is number of elements to select from that rank

# Arguments
- `archive`: The NGArchive to select elements from
- `n`: Number of elements to select

# Returns
Vector of selected NGArchiveElements
"""
function select_representative_elements(archive::NGArchive, n::Int)::Vector{NGArchiveElement}
    # Handle edge cases
    if isempty(archive.elements) || n <= 0
        return NGArchiveElement[]
    end

    if length(archive.elements) <= n
        return copy(archive.elements)
    end

    # Get elements organized by Pareto rank (O(m log m))
    ranked_elements = pareto_rank(archive)

    println(ranked_elements)

    # pause the program
    sleep(100)

    # Initialize result vector
    selected = Vector{NGArchiveElement}()

    # Find the maximum rank
    max_rank = maximum(keys(ranked_elements))

    # Select elements rank by rank (at most O(m) total elements across all ranks)
    for rank in 1:max_rank
        if !haskey(ranked_elements, rank)
            continue
        end

        remaining_slots = n - length(selected)
        if remaining_slots <= 0
            break
        end

        elements_in_rank = ranked_elements[rank]

        if length(elements_in_rank) <= remaining_slots
            # Add all elements from this rank
            append!(selected, elements_in_rank)
        else
            # Need to select a diverse subset (O(n·k) operation)
            println("before diverse selection")
            diverse_subset = select_diverse_subset(elements_in_rank, remaining_slots)
            println("after diverse selection")
            append!(selected, diverse_subset)
            break
        end

        # Safety check - if we've selected enough elements, break out of the loop
        if length(selected) >= n
            break
        end
    end

    return selected
end
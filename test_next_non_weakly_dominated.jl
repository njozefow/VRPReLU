using Pkg
Pkg.activate(".")

include("src/archive.jl")

function test_next_non_weakly_dominated()
    println("Testing next_non_weakly_dominated function...")
    
    # Test 1: Empty indices
    elements = [(1, 5.0), (2, 3.0)]
    indices = Int[]
    result = next_non_weakly_dominated(elements, indices, 1)
    println("Test 1 - Empty indices: result = $result (expected: 1)")
    @assert result == 1 "Empty indices should return input index"
    
    # Test 2: Index out of bounds
    elements = [(1, 5.0), (2, 3.0)]
    indices = [1, 2]
    result = next_non_weakly_dominated(elements, indices, 5)
    println("Test 2 - Out of bounds: result = $result (expected: 5)")
    @assert result == 5 "Out of bounds index should be returned as-is"
    
    # Test 3: No duplicates, different first objectives
    elements = [(1, 5.0), (2, 3.0), (3, 7.0)]
    indices = [1, 2, 3]
    indices_copy = copy(indices)
    result = next_non_weakly_dominated(elements, indices_copy, 1)
    println("Test 3 - No duplicates: result = $result, indices = $indices_copy")
    @assert result == 1 "Should return first index when no duplicates"
    @assert indices_copy == indices "Indices should remain unchanged"
    
    # Test 4: Exact duplicates (same objective values within tolerance)
    elements = [(5, 10.0), (5, 10.0), (5, 10.0000000000001), (6, 8.0)]
    indices = [1, 2, 3, 4]
    indices_copy = copy(indices)
    result = next_non_weakly_dominated(elements, indices_copy, 1)
    println("Test 4 - Exact duplicates: result = $result, indices = $indices_copy")
    println("  Original elements: $(elements[indices])")
    println("  Remaining elements: $([elements[i] for i in indices_copy])")
    
    # Test 5: Same first objective, different second objectives
    elements = [(5, 5.0), (5, 10.0), (5, 15.0), (6, 8.0)]
    indices = [1, 2, 3, 4]
    indices_copy = copy(indices)
    result = next_non_weakly_dominated(elements, indices_copy, 1)
    println("Test 5 - Same first obj, diff second: result = $result, indices = $indices_copy")
    println("  Should advance through all (5,*) elements")
    
    # Test 6: Mixed scenario with duplicates and different values
    elements = [(3, 10.0), (3, 10.0), (3, 15.0), (3, 15.0000000000001), (4, 5.0)]
    indices = [1, 2, 3, 4, 5]
    indices_copy = copy(indices)
    result = next_non_weakly_dominated(elements, indices_copy, 1)
    println("Test 6 - Mixed scenario: result = $result, indices = $indices_copy")
    println("  Should remove duplicates but keep different values")
    
    # Test 7: Single element
    elements = [(5, 10.0)]
    indices = [1]
    indices_copy = copy(indices)
    result = next_non_weakly_dominated(elements, indices_copy, 1)
    println("Test 7 - Single element: result = $result, indices = $indices_copy")
    @assert result == 1 "Single element should return index 1"
    
    # Test 8: Floating point precision edge case
    elements = [(5, 10.0), (5, 10.0 + 1e-13), (5, 10.0 + 1e-11)]
    indices = [1, 2, 3]
    indices_copy = copy(indices)
    result = next_non_weakly_dominated(elements, indices_copy, 1)
    println("Test 8 - Floating point precision:")
    println("  Elements: $elements")
    println("  Result: $result, indices: $indices_copy")
    println("  FLOAT_TOL = $FLOAT_TOL")
    
    println("✓ All tests completed")
end

function test_algorithmic_behavior()
    println("\nTesting algorithmic behavior...")
    
    # Create a sorted scenario like in the main algorithm
    elements = [(1, 20.0), (1, 15.0), (1, 10.0), (2, 25.0), (2, 5.0), (3, 30.0)]
    indices = collect(1:length(elements))
    
    println("Original elements with indices:")
    for (i, elem) in enumerate(elements)
        println("  Index $i: $elem")
    end
    
    # Test processing each group
    i = 1
    println("\nProcessing groups:")
    while i <= length(indices)
        println("Starting at position $i (element $(elements[indices[i]]))")
        old_indices = copy(indices)
        new_i = next_non_weakly_dominated(elements, indices, i)
        println("  Result: moved from $i to $new_i")
        println("  Indices changed from $old_indices to $indices")
        
        if new_i == i && i < length(indices)
            # Move to next different first objective
            current_obj1 = elements[indices[i]][1]
            while i <= length(indices) && elements[indices[i]][1] == current_obj1
                i += 1
            end
        else
            i = new_i
            if i <= length(indices)
                i += 1  # Move to next group
            end
        end
        
        if i > length(indices)
            break
        end
    end
end

# Run tests
test_next_non_weakly_dominated()
test_algorithmic_behavior()
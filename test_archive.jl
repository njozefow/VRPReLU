using Pkg
Pkg.activate(".")

include("src/archive.jl")

# Test helper functions
function test_dominance()
    println("Testing dominance relationships...")
    
    # Test 1: Clear dominance - a dominates b
    a = (5, 10.0)
    b = (6, 15.0)
    @assert dominates(a, b) == true "$(a) should dominate $(b)"
    @assert dominates(b, a) == false "$(b) should not dominate $(a)"
    
    # Test 2: Weak dominance - equal in one objective, better in another
    a = (5, 10.0)
    b = (5, 15.0)
    @assert dominates(a, b) == true "$(a) should weakly dominate $(b)"
    @assert dominates(b, a) == false "$(b) should not dominate $(a)"
    
    # Test 3: No dominance - trade-off
    a = (5, 15.0)
    b = (6, 10.0)
    @assert dominates(a, b) == false "$(a) should not dominate $(b)"
    @assert dominates(b, a) == false "$(b) should not dominate $(a)"
    
    # Test 4: Identical solutions
    a = (5, 10.0)
    b = (5, 10.0)
    @assert dominates(a, b) == false "Identical solutions should not dominate each other"
    @assert dominates(b, a) == false "Identical solutions should not dominate each other"
    
    # Test 5: Floating point precision
    a = (5, 10.0)
    b = (5, 10.0 + 1e-13)  # Within tolerance
    @assert dominates(a, b) == false "Should handle floating point precision"
    @assert dominates(b, a) == false "Should handle floating point precision"
    
    println("✓ Dominance tests passed")
end

function test_archive_basic()
    println("Testing basic archive operations...")
    
    # Test empty archive
    archive = Archive()
    @assert length(archive.elements) == 0 "Empty archive should have 0 elements"
    
    # Test adding elements
    add_to_archive!(archive, 5, 10.0)
    @assert length(archive.elements) == 1 "Archive should have 1 element"
    @assert archive.elements[1] == (5, 10.0) "Element should match"
    
    add_to_archive!(archive, 6, 8.0)
    @assert length(archive.elements) == 2 "Archive should have 2 elements"
    
    println("✓ Basic archive tests passed")
end

function test_ranking_simple()
    println("Testing simple ranking scenarios...")
    
    # Test 1: Single front (all non-dominated)
    archive = Archive()
    add_to_archive!(archive, 1, 10.0)  # Better in first objective
    add_to_archive!(archive, 5, 2.0)   # Better in second objective  
    add_to_archive!(archive, 3, 5.0)   # Middle solution
    
    fronts = rank(archive)
    @assert length(fronts) == 1 "Should have 1 front"
    @assert length(fronts[1]) == 3 "Front should have 3 elements"
    
    # Test 2: Two clear fronts
    archive = Archive()
    # Front 1: Non-dominated
    add_to_archive!(archive, 1, 10.0)
    add_to_archive!(archive, 5, 2.0)
    # Front 2: Dominated by front 1
    add_to_archive!(archive, 6, 8.0)
    add_to_archive!(archive, 3, 12.0)
    
    fronts = rank(archive)
    @assert length(fronts) == 2 "Should have 2 fronts"
    @assert length(fronts[1]) == 2 "First front should have 2 elements"
    @assert length(fronts[2]) == 2 "Second front should have 2 elements"
    
    println("✓ Simple ranking tests passed")
end

function test_duplicates()
    println("Testing duplicate element handling...")
    
    # Test exact duplicates
    archive = Archive()
    add_to_archive!(archive, 5, 10.0)
    add_to_archive!(archive, 5, 10.0)  # Exact duplicate
    add_to_archive!(archive, 5, 10.0 + 1e-13)  # Within tolerance
    add_to_archive!(archive, 6, 8.0)
    
    fronts = rank(archive)
    println("Archive elements: ", archive.elements)
    println("Fronts: ", fronts)
    
    # Should handle duplicates without crashing
    @assert length(fronts) >= 1 "Should produce at least one front"
    
    println("✓ Duplicate handling tests passed")
end

function test_edge_cases()
    println("Testing edge cases...")
    
    # Test 1: Empty archive
    archive = Archive()
    fronts = rank(archive)
    @assert length(fronts) == 0 "Empty archive should produce no fronts"
    
    # Test 2: Single element
    archive = Archive()
    add_to_archive!(archive, 5, 10.0)
    fronts = rank(archive)
    @assert length(fronts) == 1 "Single element should produce 1 front"
    @assert length(fronts[1]) == 1 "Front should have 1 element"
    @assert fronts[1][1] == 1 "Should reference first element"
    
    # Test 3: All identical elements
    archive = Archive()
    for i in 1:5
        add_to_archive!(archive, 5, 10.0)
    end
    fronts = rank(archive)
    println("Identical elements fronts: ", fronts)
    
    println("✓ Edge case tests passed")
end

function test_weak_dominance_detailed()
    println("Testing detailed weak dominance scenarios...")
    
    # Scenario 1: Equal first objective, different second
    archive = Archive()
    add_to_archive!(archive, 5, 10.0)  # Should dominate next
    add_to_archive!(archive, 5, 15.0)  # Dominated
    add_to_archive!(archive, 5, 5.0)   # Should dominate first two
    
    fronts = rank(archive)
    println("Weak dominance scenario 1 - fronts: ", fronts)
    @assert length(fronts) >= 2 "Should have multiple fronts for dominated solutions"
    
    # Scenario 2: Equal second objective, different first  
    archive = Archive()
    add_to_archive!(archive, 10, 5.0)  # Dominated
    add_to_archive!(archive, 5, 5.0)   # Should dominate
    add_to_archive!(archive, 15, 5.0)  # Dominated
    
    fronts = rank(archive)
    println("Weak dominance scenario 2 - fronts: ", fronts) 
    @assert length(fronts) >= 2 "Should have multiple fronts for dominated solutions"
    
    println("✓ Detailed weak dominance tests passed")
end

function test_floating_point_precision()
    println("Testing floating point precision...")
    
    archive = Archive()
    base_val = 10.0
    
    # Add values very close to each other
    add_to_archive!(archive, 5, base_val)
    add_to_archive!(archive, 5, base_val + 1e-13)  # Within tolerance
    add_to_archive!(archive, 5, base_val + 1e-11)  # Outside tolerance
    add_to_archive!(archive, 5, base_val - 1e-13)  # Within tolerance
    add_to_archive!(archive, 5, base_val - 1e-11)  # Outside tolerance
    
    fronts = rank(archive)
    println("Floating point precision - elements: ", archive.elements)
    println("Floating point precision - fronts: ", fronts)
    
    # Should handle precision issues without infinite loops
    @assert length(fronts) >= 1 "Should produce at least one front"
    
    println("✓ Floating point precision tests passed")
end

function test_complex_scenario()
    println("Testing complex multi-front scenario...")
    
    archive = Archive()
    
    # Create a complex scenario with multiple fronts
    # Front 1: Pareto optimal
    add_to_archive!(archive, 1, 20.0)
    add_to_archive!(archive, 5, 5.0) 
    add_to_archive!(archive, 3, 10.0)
    
    # Front 2: Dominated by front 1 but not each other
    add_to_archive!(archive, 2, 15.0)
    add_to_archive!(archive, 4, 12.0)
    add_to_archive!(archive, 6, 8.0)
    
    # Front 3: Dominated by front 2
    add_to_archive!(archive, 7, 18.0)
    add_to_archive!(archive, 4, 20.0)
    
    fronts = rank(archive)
    println("Complex scenario - fronts: ", fronts)
    println("Total elements: ", length(archive.elements))
    println("Total fronts: ", length(fronts))
    
    # Verify all elements are assigned to a front
    total_assigned = sum(length(front) for front in fronts)
    @assert total_assigned == length(archive.elements) "All elements should be assigned to fronts"
    
    println("✓ Complex scenario tests passed")
end

function run_all_tests()
    println("Running comprehensive archive tests...\n")
    
    try
        test_dominance()
        test_archive_basic()
        test_ranking_simple()
        test_duplicates()
        test_edge_cases()
        test_weak_dominance_detailed()
        test_floating_point_precision()
        test_complex_scenario()
        
        println("\n🎉 All tests passed successfully!")
        
    catch e
        println("\n❌ Test failed with error: ", e)
        rethrow(e)
    end
end

# Run the tests
run_all_tests()
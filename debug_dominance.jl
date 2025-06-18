using Pkg
Pkg.activate(".")

include("src/archive.jl")

println("Debug weak dominance...")

# Test elements
a = (5, 5.0)   # Should be best
b = (5, 10.0)  # Should be middle  
c = (5, 15.0)  # Should be worst

println("Elements:")
println("a = ", a, " (should be best)")
println("b = ", b, " (should be middle)")  
println("c = ", c, " (should be worst)")

println("\nDominance relationships:")
println("dominates(a, b) = ", dominates(a, b), " (should be true)")
println("dominates(a, c) = ", dominates(a, c), " (should be true)")
println("dominates(b, c) = ", dominates(b, c), " (should be true)")
println("dominates(b, a) = ", dominates(b, a), " (should be false)")
println("dominates(c, a) = ", dominates(c, a), " (should be false)")
println("dominates(c, b) = ", dominates(c, b), " (should be false)")

# Test with archive
archive = Archive()
add_to_archive!(archive, 5, 5.0)   # Element 1 (best)
add_to_archive!(archive, 5, 10.0)  # Element 2 (middle)
add_to_archive!(archive, 5, 15.0)  # Element 3 (worst)

println("\nArchive elements:")
for (i, elem) in enumerate(archive.elements)
    println("Element $i: ", elem)
end

# Check sorting step
n = length(archive.elements)
indices = collect(1:n)
println("\nBefore sorting - indices: ", indices)
println("Elements by index: ", [archive.elements[i] for i in indices])

sort!(indices, by=i -> (archive.elements[i][1], archive.elements[i][2]))
println("After sorting - indices: ", indices)
println("Elements by sorted index: ", [archive.elements[i] for i in indices])

fronts = rank(archive)
println("\nRanking result:")
for (i, front) in enumerate(fronts)
    println("Front $i: ", front)
    println("  Elements: ", [archive.elements[idx] for idx in front])
end
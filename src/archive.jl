struct Archive
    elements::Vector{Tuple{Int64,Float64}}

    Archive() = new(Vector{Tuple{Int64,Float64}}())
end

@inline push!(archive::Archive, objone::Int64, objtwo::Float64) = Base.push!(archive.elements, (objone, objtwo))



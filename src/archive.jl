struct Archive
    elements::Vector{Tuple{Int,Float64}}  # (picost, cost)

    Archive() = new(Vector{Tuple{Int,Float64}}())
end

const FLOAT_TOL = 1e-6

@inline approx_leq(a::Float64, b::Float64) = a <= b + FLOAT_TOL

function add_point!(archive::Archive, distance::Int, picost::Float64)
    elements = archive.elements

    if isempty(elements)
        push!(elements, (distance, picost))
        return
    end

    insert_pos = searchsortedfirst(elements, distance, by=elem -> elem[1])

    if insert_pos <= length(elements)
        @inbounds d, pc = elements[insert_pos]
        # Same point or weakly dominated
        d <= distance && approx_leq(pc, picost) && return
    end

    for i in (insert_pos-1):-1:1
        @inbounds d, pc = elements[i]

        if pc > picost + FLOAT_TOL
            break
        elseif pc <= picost + FLOAT_TOL
            return
        end
    end

    to_remove = Int[]

    for i in insert_pos:length(elements)
        @inbounds d, pc = elements[i]

        if picost <= pc + FLOAT_TOL
            push!(to_remove, i)
        else
            break
        end
    end

    for i in reverse(to_remove)
        deleteat!(elements, i)
    end

    insert!(elements, insert_pos, (distance, picost))
end
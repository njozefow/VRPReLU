"""
    ngin(ng, j, u, node)::Bool

Check if a node is in the neighborhood group `ng` for a given position `j` and bitmask `u`.

# Arguments
- `ng`: The neighborhood group matrix.
- `j`: The position in the neighborhood group.
- `u`: The bitmask representing the nodes in the neighborhood group.
- `node`: The node to check.

# Returns
- `Bool`: `true` if the node is in the neighborhood group, `false` otherwise.
"""
function ngin(ng, j, u, node)
    @inbounds for idx in 1:7
        if u == 0
            return false
        elseif u & 1 == 1 && @inbounds ng[idx, j] == node
            return true
        else
            u >>= 1
        end
    end

    return false
end

"""
    ngintersect(ng, i, u, j)::UInt8

Compute the intersection of two neighborhood groups.

# Arguments
- `ng`: The neighborhood group matrix.
- `i`: The first neighborhood group.
- `u`: The bitmask for the first neighborhood group.
- `j`: The second neighborhood group.

# Returns
- `UInt8`: The bitmask representing the intersection of the two neighborhood groups.
"""
function ngintersect(ng, i, u, j)::UInt8
    v::UInt8 = 0

    for posj in reverse(1:7)
        @inbounds node = ng[posj, j]

        if i == node
            v |= UInt8(1)
        elseif ngin(ng, i, u, node)
            v |= UInt8(1)
        end

        v <<= 1
    end

    v >>= 1

    return v
end

"""
    compatible(ng, i, ssng, u)::Bool

Check if two neighborhood groups are compatible.

# Arguments
- `ng`: The neighborhood group matrix.
- `i`: The first neighborhood group.
- `ssng`: The bitmask for the second neighborhood group.
- `u`: The bitmask for the first neighborhood group.

# Returns
- `Bool`: `true` if the neighborhood groups are compatible, `false` otherwise.
"""
function compatible(ng, i, ssng, u)
    @inbounds for idx in 1:7
        if ssng == 0
            return true
        elseif ssng & 1 == 1 && in(ng[idx, i], u)
            return false
        else
            ssng >>= 1
        end
    end

    return true
end

"""
    ngsubsetequal(ngone, uone::UInt8, ngtwo, utwo::UInt8, node)::Bool

Determine if the set represented by `ngone` and `uone` is a subset of the set represented by `ngtwo` and `utwo` for a given `node`.

# Arguments
- `ngone`: The first set of nodes.
- `uone::UInt8`: The bitmask for the first set of nodes.
- `ngtwo`: The second set of nodes.
- `utwo::UInt8`: The bitmask for the second set of nodes.
- `node`: The node to compare.

# Returns
- `Bool`: `true` if `ngone` is a subset of `ngtwo`, `false` otherwise.
"""
function ngsubsetequal(ngone, uone::UInt8, ngtwo, utwo::UInt8, node)
    posone = 1
    postwo = 1

    @inbounds while posone <= 7 && uone != 0
        nodeone = ngone[posone, node]

        if uone & 1 == 1
            while postwo <= 7 && utwo != 0 && ngtwo[postwo, node] < nodeone
                utwo >>= 1
                postwo += 1
            end

            nodetwo = ngtwo[postwo, node]
            if postwo > 7 || utwo == 0 || (nodeone == nodetwo && utwo & 1 == 0)
                return false
            end
        end

        posone += 1
        uone >>= 1
    end

    return true
end

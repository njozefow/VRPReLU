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

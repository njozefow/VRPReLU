import .VRPReLU: delta

const MAX_NG_SIZE = delta - 1

function ngin(ng, j, u, node)
    @fastmath @inbounds for idx in 1:MAX_NG_SIZE
        if u == 0
            return false
        elseif u & 1 == 1 && @fastmath @inbounds ng[idx, j] == node
            return true
        else
            u >>= 1
        end
    end

    return false
end

# function ngintersect(ng, i, u, j)::UInt8
#     v::UInt8 = 0

#     for posj in reverse(1:MAX_NG_SIZE)
#         @inbounds node = ng[posj, j]

#         if i == node
#             v |= UInt8(1)
#         elseif ngin(ng, i, u, node)
#             v |= UInt8(1)
#         end

#         v <<= 1
#     end

#     v >>= 1

#     return v
# end

# TODO: Il faudra vérifier si cette fonction est plus rapide que l'ancienne et si elle est correcte
@inline function ngintersect(ng, i, u::UInt8, j)::UInt8
    ngintersect(ng, i, u, ng, j)
end

function ngintersect(ngfrom, from, ufrom::UInt8, ngto, to)::UInt8
    result::UInt8 = 0
    posfrom = 1

    @fastmath @inbounds for posto in 1:MAX_NG_SIZE
        node_to = ngto[posto, to]

        # Direct match with source index
        if node_to == from
            result |= UInt8(1) << (posto - 1)
            continue
        end

        # Early exit if we've processed all source nodes
        node_to > from && ufrom == 0 && break

        # Find matching node in source group
        while posfrom ≤ MAX_NG_SIZE && ufrom ≠ 0
            node_from = ngfrom[posfrom, from]

            if node_from == node_to
                if ufrom & 1 == 1
                    result |= UInt8(1) << (posto - 1)
                end
                ufrom >>= 1
                posfrom += 1
                break
            elseif node_from < node_to
                ufrom >>= 1
                posfrom += 1
            else
                break
            end
        end
    end

    return result
end

# function ngintersect(ngfrom, ufrom::UInt8, from, ngto, to)::UInt8
#     result::UInt8 = 0
#     posfrom = 1

#     @fastmath @inbounds for posto in 1:MAX_NG_SIZE
#         node = ngto[posto, to]

#         if node == from
#             result |= UInt8(1) << (posto - 1)
#         elseif node > from && ufrom == 0
#             break
#         else
#             while posfrom <= MAX_NG_SIZE && ufrom != 0 && ngfrom[posfrom, from] < node
#                 ufrom >>= 1
#                 posfrom += 1
#             end

#             if posfrom <= MAX_NG_SIZE && ufrom != 0 && ngfrom[posfrom, from] == node
#                 result |= UInt8(1) << (posto - 1)
#             end
#         end
#     end

#     return result
# end

function compatible(ng, i, ssng, u)
    @inbounds for idx in 1:MAX_NG_SIZE
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

# function ngsubsetequal(ngone::Matrix{Int}, uone::UInt8, ngtwo::Matrix{Int}, utwo::UInt8, node::Int)::Bool
#     # Early exit if no elements to check
#     uone == 0 && return true
#     utwo == 0 && return false

#     posone = 1
#     postwo = 1

#     @fastmath @inbounds while posone <= MAX_NG_SIZE && uone != 0
#         # Skip inactive elements in first set
#         if uone & 1 == 0
#             posone += 1
#             uone >>= 1
#             continue
#         end

#         nodeone = ngone[posone, node]

#         # Advance through second set until we find matching or greater element
#         while postwo <= MAX_NG_SIZE && utwo != 0 && ngtwo[postwo, node] < nodeone
#             utwo >>= 1
#             postwo += 1
#         end

#         # Check failure conditions:
#         # 1. Ran out of elements in second set
#         # 2. Found matching element but it's not active
#         # 3. Current element in second set is greater (element not found)
#         if postwo > MAX_NG_SIZE ||
#            utwo == 0 ||
#            ngtwo[postwo, node] > nodeone ||
#            (nodeone == ngtwo[postwo, node] && utwo & 1 == 0)
#             return false
#         end

#         posone += 1
#         uone >>= 1
#     end

#     return true
# end

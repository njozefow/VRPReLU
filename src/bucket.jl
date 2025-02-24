@inline isend(bucket, i) = i > length(bucket)

function push(sp, bucket, label::Label, dom)
    pos = 1

    label_reduced_cost = path_reduced_cost(label)

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if path_reduced_cost(l) >= label_reduced_cost - myeps
            break
        elseif dom(sp, l, label)
            return false
        else
            pos += 1
        end
    end

    # TODO: traiter les labels egaux

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if dom(sp, label, l)
            break
        else
            deleteat!(bucket, pos)
        end
    end

    addpos = pos
    pos += 1

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if dom(sp, label, l)
            deleteat!(bucket, pos)
        else
            pos += 1
        end
    end

    insert!(bucket, addpos, label)

    return true
end

# Check if label is dominated by a label in the bucket
function dominate(sp, bucket, label::Label, dom)
    pos = 1

    label_reduced_cost = path_reduced_cost(label)

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if path_reduced_cost(l) > label_reduced_cost - myeps
            break
        elseif dom(sp, l, label)
            return true
        else
            pos += 1
        end
    end

    return false
end

# Remove labels in bucket dominated by label
function clean(sp, bucket, label::Label, dom)
    pos = length(bucket)

    label_reduced_cost = path_reduced_cost(label)

    while pos > 0
        @inbounds l = bucket[pos]

        if path_reduced_cost(l) < label_reduced_cost + myeps
            break
        elseif dom(sp, label, l)
            deleteat!(bucket, pos)
        end

        pos -= 1
    end
end
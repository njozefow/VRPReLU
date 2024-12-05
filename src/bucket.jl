@inline isend(bucket, i) = i > length(bucket)

function push(bucket, label::Label)
    pos = 1

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if l.reduced_cost >= label.reduced_cost - myeps
            break
        elseif dominates(l, label)
            return false
        else
            pos += 1
        end
    end

    # TODO: traiter les labels egaux

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if !dominates(label, l)
            break
        else
            deleteat!(bucket, pos)
        end
    end

    addpos = pos
    pos += 1

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if dominates(label, l)
            deleteat!(bucket, pos)
        else
            pos += 1
        end
    end

    insert!(bucket, addpos, label)

    return true
end

# Check if label is dominated by a label in the bucket
function dominate(bucket, label::Label)
    pos = 1

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if l.reduced_cost > label.reduced_cost - myeps
            break
        elseif dominates(l, label)
            return true
        else
            pos += 1
        end
    end

    return false
end

# Remove labels in bucket dominated by label
function clean(bucket, label::Label)
    pos = length(bucket)

    while pos > 0
        @inbounds l = bucket[pos]

        if l.reduced_cost < label.reduced_cost + myeps
            break
        elseif dominates(label, l)
            deleteat!(bucket, pos)
        end

        pos -= 1
    end
end
@inline isend(bucket, i) = i > length(bucket)

function push(sp, bucket, label::Label, dominates)
    pos = 1

    label_reduced_cost = path_reduced_cost(sp, label)

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if path_reduced_cost(sp, l) >= label_reduced_cost - myeps
            break
        elseif dominates(l, label)
            return false
        else
            pos += 1
        end
    end

    # Labels with the same reduced cost -> dominate, dominated, or equal
    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if abs(path_reduced_cost(sp, l) - label_reduced_cost) >= myeps
            break
        elseif equal(label, l) || dominates(l, label)
            return false
        elseif dominates(label, l)
            deleteat!(bucket, pos)
        else
            pos += 1
        end
    end

    # First non dominated label
    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        # TODO: Il faudra vérifier si l'égalité est correcte
        if equal(label, l)
            return false
        elseif dominates(label, l)
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
function check_dominate(sp, bucket, label::Label, dominates)
    pos = 1

    label_reduced_cost = path_reduced_cost(sp, label)

    while !isend(bucket, pos)
        @inbounds l = bucket[pos]

        if path_reduced_cost(sp, l) > label_reduced_cost - myeps
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
function clean_bucket(sp, bucket, label::Label, dominates)
    pos = length(bucket)

    label_reduced_cost = path_reduced_cost(sp, label)

    while pos > 0
        @inbounds l = bucket[pos]

        if path_reduced_cost(sp, l) < label_reduced_cost + myeps
            break
        elseif dominates(label, l)
            deleteat!(bucket, pos)
        end

        pos -= 1
    end
end
module VRPReLU

using EzXML
using JuMP
using Gurobi
using DataFrames
using Statistics
# using DataStructures
using CSV

import Base: isempty, copy, length, push!, empty!, isempty

const GRB_ENV_REF = Ref{Gurobi.Env}()

const TIME_LIMIT = 3600.0

const GAP_EPS = 1.04

const delta = 8
const n_psng = 128
const myeps = 1e-4
const max_heuristic_routes = 30
const max_exact_routes = 150
const nb_buckets = 4

include("param.jl")
include("instance.jl")
include("column.jl")
include("master.jl")
include("archive.jl")
include("ngarchive.jl")
include("subproblem.jl")
include("ngset.jl")
include("label.jl")
include("heuristic.jl")
include("bucket.jl")
include("ngdynprog.jl")
include("colgen.jl")
include("searchtree.jl")
# include("tbound.jl")
include("paretobound.jl")
include("enumlabel.jl")
include("enumeration.jl")

@kwdef mutable struct Result
    # Lower bound computation
    iteration::Int = 0
    heur_iteration::Int = 0
    dp_iteration::Int = 0
    lower_bound::Float64 = Inf
    nbcolumns_elem_lb::Int = 0
    nbcolumns_nonelem_lb::Int = 0
    lb_cpu_time::Float64 = Inf
    isint::Bool = false

    # enumeration
    nbcolumns_enum::Int = 0
    enum_cpu_time::Float64 = Inf

    # optimum
    upper_bound::Int = -typemax(Int)
    gap::Float64 = Inf
    upper_bound_cpu_time::Float64 = Inf

    # Total algorithm
    totaltime::Float64 = Inf
    complete::Bool = false
end

function __init__()
    global GRB_ENV_REF
    GRB_ENV_REF[] = Gurobi.Env()
    return
end

# function branch_and_price(param, master, sp, lower_bound, lb_cpu_time)
#     upperbound, ubtime, _ = search_tree(param, master, sp)

#     ttime = cpu_time(param)

#     if upperbound == typemax(Int)
#         return Result(lower_bound, true, lb_cpu_time, false, 0.0, Inf, Inf, ttime, false)
#     else
#         return Result(lower_bound, true, lb_cpu_time, false, round(upperbound), (upperbound - lower_bound) / lower_bound, ubtime, ttime, !timeout(param))
#     end
# end

# function solve_master(param, master, lower_bound, lb_cpu_time)
#     delete_ngroutes(master)
#     switch_to_integer(master)
#     optimize(master, remaining_time(param))

#     ttime = cpu_time(param)
#     upperbound = round(objective_value(master))
#     ubtime = ttime

#     if status(master) == MOI.OPTIMAL
#         return Result(lower_bound, true, lb_cpu_time, false, upperbound, (upperbound - lower_bound) / lower_bound, ubtime, ttime, !timeout(param))
#     else
#         return Result(lower_bound, false, lb_cpu_time, false, upperbound, (upperbound - lower_bound) / lower_bound, ubtime, ttime, !timeout(param))
#     end
# end

function enumerate(param, master, sp, result)
    print("Enumeration\n")

    lower_bound = result.lower_bound
    lb_cpu_time = result.lb_cpu_time

    delete_ngroutes(master)
    optimize(master, remaining_time(param))
    set(sp, master.constraints)
    # compute_tbounds(sp)
    compute_pbounds(sp)

    upperbound = floor(lower_bound * GAP_EPS + myeps)
    sp.gap = upperbound - lower_bound

    if timeout(param)
        # return Result(lower_bound, true, lb_cpu_time, false, 0.0, Inf, Inf, param.timelimit, false)
        return result
    end

    routes = enumeration(param, sp)

    return result

    if timeout(param)
        # TODO: récupérer la meilleur solution trouvée s'il y en a une
        # return Result(lower_bound, true, lb_cpu_time, false, 0.0, Inf, Inf, param.timelimit, false)
        return result
    end

    result.nbcolumns_enum = length(routes)
    result.enum_cpu_time = cpu_time(param) - lb_cpu_time
    # TODO: Je me suis arrêté ici

    switch_to_integer(master)
    push_elementary_column(master, routes)

    # bcmaster(sp.instance, master)

    optimize(master, remaining_time(param))

    # TODO: récupérer la meilleure borne sup s'il y en a une
    if timeout(param)
        result.total_time = param.timelimit
        return result
        # return Result(lower_bound, true, lb_cpu_time, false, 0.0, Inf, Inf, param.timelimit, false)
    end

    result.upper_bound = round(objective_value(master))

    result.gap = (result.upper_bound - result.lower_bound) / result.lower_bound

    # TODO: récupérer le temps de la borne sup
    result.upper_bound_cpu_time = cpu_time(param) - result.lb_cpu_time - result.enum_cpu_time
    result.totaltime = cpu_time(param)
    result.complete = true

    # return Result(lower_bound, true, lb_cpu_time, false, round(upperbound), (upperbound - lower_bound) / lower_bound, ubtime, ttime, !timeout(param))
end

function solve(instance::Instance)
    # Set parameters
    # param = Param(300.0)
    param = Param(TIME_LIMIT)

    # Build master problem
    master = Master(instance)
    add_singleton(master, instance)
    add_pairs(master, instance)

    # Build data structure for subproblem to avoid allocation
    sp = Subproblem(instance)

    result = Result()

    # Apply column generation to compute lower bound
    result.iteration, result.dp_iteration = colgen(param, master, sp)
    result.heur_iteration = result.iteration - result.dp_iteration
    # lower_bound = objective_value(master)
    # lb_cpu_time = cpu_time(param)

    # Two possibilities:
    # 1. timeout -> end
    # 2. Lower bound is found -> continue

    if timeout(param)
        return result
        # return Result(0, false, lb_cpu_time, false, typemax(Int), Inf, Inf, lb_cpu_time, false)
    end

    # unset_silent(master.model)
    # optimize(master)

    # optimize(master, remaining_time(param))

    result.lower_bound = Float64(objective_value(master))
    result.nbcolumns_elem_lb = length(master.columns)
    result.nbcolumns_nonelem_lb = length(master.ngcolumns)
    result.lb_cpu_time = cpu_time(param)

    if isinteger(master)
        result.isint = true
        result.enum_cpu_time = 0
        result.upper_bound = round(result.lower_bound + myeps)
        result.gap = 0
        result.upper_bound_cpu_time = result.lb_cpu_time
        result.totaltime = result.lb_cpu_time
        result.complete = true
        # return Result(lower_bound, true, lb_cpu_time, true, round(lower_bound), 0, lb_cpu_time, lb_cpu_time, true)

        # lengthstar = [master.columns[i].cost for i in eachindex(master.columns) if value(master.x[i]) > 0.5]
        # result.max_length = maximum(lengthstar)
        # result.nb_vehicles = length(lengthstar)

        return result
    end

    # branch_and_price
    # return branch_and_price(param, master, sp, lower_bound, lb_cpu_time)

    # just solving the master
    # return solve_master(param, master, lower_bound, lb_cpu_time)

    # TODO: a remettre
    enumerate(param, master, sp, result)

    return result
end

function solve(path::String, type::String, name::AbstractString, ratio=0.75)
    instance = read(joinpath(path, type, "$name.xml"), joinpath(path, type, "$name.relu"), ratio)
    return solve(instance)
end

# TODO: Il faudra faire varier le ratio
function solve(path::String, type::String, ratio=0.75)
    # df = DataFrame(name=String[], lower_bound=Float64[], cg_ended=Bool[], lb_cpu_time=Float64[], isint=Bool[], upper_bound=Int[], gap=Float64[], b_cpu_time=Float64[], totaltime=Float64[], complete=Bool[])
    df = DataFrame(name=String[], nbv=Int[], soft=Int[], hard=Int[], lb=Float64[], it=Int[], h_it=Int[], dp_it=Int[], nbc_elem_lb=Int[], nbc_nonelem_lb=Int[], lb_cpu_time=Float64[], isint=Bool[], nbc_enum=Int64[], cpu_time=Float64[], ub=Int[], gap=Float64[], ub_cpu_time=Float64[], time=Float64[], complete=Bool[])

    path = joinpath(path, type)

    # for file in readdir(path)
    #     if occursin(".xml", file) && (occursin("025", file))
    #         for ratio in [0.25, 0.5, 0.75, 1]
    #             name = split(file, ".")[1]
    #             filename = joinpath(path, "$name.xml")
    #             filerelu = joinpath(path, "$name.relu")
    #             println(joinpath(path, "$name.xml"))
    #             flush(stdout)
    #             instance = read(filename, filerelu, ratio)
    #             res = solve(instance)

    #             push!(df, (name, n_vehicles(instance), soft_distance_limit(instance), hard_distance_limit(instance), res.lower_bound, res.iteration, res.heur_iteration, res.dp_iteration, res.nbcolumns_elem_lb, res.nbcolumns_nonelem_lb, res.lb_cpu_time, res.isint, res.nbcolumns_enum, res.enum_cpu_time, res.upper_bound, round(res.gap, digits=2), res.upper_bound_cpu_time, res.totaltime, res.complete))
    #         end
    #     end
    # end

    for file in readdir(path)
        if occursin(".xml", file) && (occursin("050", file))
            for ratio in [0.0, 0.25, 0.5, 0.75, 1]
                # for ratio in [0.0]
                name = split(file, ".")[1]
                filename = joinpath(path, "$name.xml")
                filerelu = joinpath(path, "$name.relu")
                println(joinpath(path, "$name.xml"))
                flush(stdout)
                instance = read(filename, filerelu, ratio)
                res = solve(instance)

                push!(df, (name, n_vehicles(instance), soft_distance_limit(instance), hard_distance_limit(instance), res.lower_bound, res.iteration, res.heur_iteration, res.dp_iteration, res.nbcolumns_elem_lb, res.nbcolumns_nonelem_lb, res.lb_cpu_time, res.isint, res.nbcolumns_enum, res.enum_cpu_time, res.upper_bound, round(res.gap, digits=2), res.upper_bound_cpu_time, res.totaltime, res.complete))
            end
        end
    end

    println(df)

    println(describe(df))

    # print average of the columns : nbc_elem_lb, nbc_nonelem_lb, lb_cpu_time
    # println("nbc_elem_lb: ", mean(df.nbc_elem_lb))
    # println("nbc_nonelem_lb: ", mean(df.nbc_nonelem_lb))
    # println("lb_cpu_time: ", mean(df.lb_cpu_time))

    CSV.write(type * ".csv", df)
end

function run(path="bench/solomon", type="R1")
    # Dummy run
    println("Dummy run")
    name = type * "01_025"
    filename = joinpath(path, type, "$name.xml")
    filerelu = joinpath(path, type, "$name.relu")
    instance = read(filename, filerelu, 0.75)
    solve(instance)

    # TODO: A retirer
    # return

    solve(path, type)

    return 0
end

end # module BapVRPTW

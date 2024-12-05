struct Param
    timelimit::Float64

    start_time::Float64
end

function Param(timelimit::Float64=3600.0)
    start_time = time()

    return Param(timelimit, start_time)
end

timeout(param::Param) = time() - param.start_time >= param.timelimit
remaining_time(param::Param) = max(0.0, param.timelimit - cpu_time(param))
cpu_time(param::Param) = round(time() - param.start_time, digits=2)

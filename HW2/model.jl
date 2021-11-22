
function dispatchProblem(generators, periods, generators_data, demand)
    # Extract relevant data
    startup_categories = Dict(g => 1:length(gen["startup"]) for (g,gen) in generators_data)
    pwl_points = Dict(g => 1:length(gen["piecewise_production"]) for (g,gen) in generators_data)

    dispatch_problem = Model(() -> Gurobi.Optimizer(GUROBI_ENV))
    set_optimizer_attribute(dispatch_problem, "MIPGap", 1e-3)

    # Generators variables
    @variable(dispatch_problem, commit[generators, periods], Bin)
    @variable(dispatch_problem, startup[generators, periods], Bin)
    @variable(dispatch_problem, shutdown[generators, periods], Bin)
    @variable(dispatch_problem, 0 <= prod[generators, periods])
    @variable(dispatch_problem, delta[g in generators, startup_categories[g], periods], Bin)
    @variable(dispatch_problem, cost[generators, periods])
    @variable(dispatch_problem, 0 <= pwl_prod[g in generators, pwl_points[g], periods] <= 1)
    @variable(dispatch_problem, total_cost[generators])

    # Objective - minimize production cost
    @objective(dispatch_problem, Min,
        sum(total_cost[g] for g in generators)  # for sake of clarity, we define the total cost of each gen separatly
    )

    @constraint(dispatch_problem, [g in generators],
        total_cost[g] >= sum(cost[g, t] + generators_data[g]["piecewise_production"][1]["cost"] * commit[g, t]
        + sum(delta[g, s, t] * generators_data[g]["startup"][s]["cost"] for s in startup_categories[g])
        for t in periods)
    )

    # Market-clearing constraint
    @constraint(dispatch_problem, MC_cons[t in periods],
        sum(generators_data[g]["power_output_minimum"] * commit[g, t] + prod[g, t] for g in generators) == demand[t]
    )

    # Generator constraints
    @constraint(dispatch_problem, Pmax_cons1[g in generators, t in periods],  # Pmax when startup
        prod[g, t] <= (generators_data[g]["power_output_maximum"]
        - generators_data[g]["power_output_minimum"]) * commit[g, t]
        - max(generators_data[g]["power_output_maximum"]
        - generators_data[g]["ramp_startup_limit"], 0.0) * startup[g, t]
    )

    @constraint(dispatch_problem, Pmax_cons2[g in generators, t in periods, t2 in periods; t2-t==1],  # Pmax when shutdown
    prod[g, t] <= (generators_data[g]["power_output_maximum"]
    - generators_data[g]["power_output_minimum"]) * commit[g, t]
    - max(generators_data[g]["power_output_maximum"]
    - generators_data[g]["ramp_shutdown_limit"], 0.0) * shutdown[g, t2]
    )

    @constraint(dispatch_problem, [g in generators, t1 in periods, t2 in periods; t2-t1==1],
        startup[g, t2] - shutdown[g, t2] == commit[g, t2] - commit[g, t1]
    )

    @constraint(dispatch_problem, [g in generators, t in periods; t==1],
    startup[g, t] - shutdown[g, t] == commit[g, t] - generators_data[g]["unit_on_t0"]
    )

    @constraint(dispatch_problem, [g in generators, t in periods; t==1],
    generators_data[g]["unit_on_t0"] *
    (generators_data[g]["power_output_t0"] - generators_data[g]["power_output_minimum"])
    <= generators_data[g]["unit_on_t0"] *
    (generators_data[g]["power_output_maximum"] - generators_data[g]["power_output_minimum"])
    - max(generators_data[g]["power_output_maximum"] - generators_data[g]["ramp_shutdown_limit"], 0) * shutdown[g, t]
    )

    # min up and down time
    @constraint(dispatch_problem, min_up_time[g in generators, t in periods; t>=min(generators_data[g]["time_up_minimum"], maximum(periods))],
        sum(startup[g, t2] for t2 in periods
        if (t2 >= t - min(generators_data[g]["time_up_minimum"], maximum(periods))+1 && t2<=t))
            <= commit[g, t]
    )

    @constraint(dispatch_problem, min_down_time[g in generators, t in periods; t>=min(generators_data[g]["time_down_minimum"], maximum(periods))],
        sum(shutdown[g, t2] for t2 in periods
        if (t2 >= t - min(generators_data[g]["time_down_minimum"], maximum(periods))+1 && t2<=t))
            <= 1 - commit[g, t]
    )

    for g in generators
        if generators_data[g]["unit_on_t0"] == 1  # min up time if generator is on in t=0
            @constraint(dispatch_problem,
                sum(commit[g, t] - 1 for t in periods if (t <= min(generators_data[g]["time_up_minimum"]-generators_data[g]["time_up_t0"], maximum(periods)))) == 0
            )
        else  # min down time if generator is off in t=0
            @constraint(dispatch_problem,
                sum(commit[g, t] for t in periods if (t <= min(generators_data[g]["time_down_minimum"]-generators_data[g]["time_down_t0"], maximum(periods)))) == 0
            )
        end
    end

    # ramp constraints
    @constraint(dispatch_problem, rampup[g in generators, t1 in periods, t2 in periods; t2-t1==1],
        prod[g, t2] - prod[g, t1] <= generators_data[g]["ramp_up_limit"]
    )

    @constraint(dispatch_problem, rampupinit[g in generators, t in periods; t==1],
        prod[g, t] - generators_data[g]["unit_on_t0"] *
        (generators_data[g]["power_output_t0"] - generators_data[g]["power_output_minimum"])
        <= generators_data[g]["ramp_up_limit"] # initial condition
    )

    @constraint(dispatch_problem, rampdown[g in generators, t1 in periods, t2 in periods; t2-t1==1],
        - prod[g, t2] + prod[g, t1] <= generators_data[g]["ramp_down_limit"]
    )

    @constraint(dispatch_problem, rampdowninit[g in generators, t in periods; t==1],
        - prod[g, t] + generators_data[g]["unit_on_t0"] *
        (generators_data[g]["power_output_t0"] - generators_data[g]["power_output_minimum"])
        <= generators_data[g]["ramp_down_limit"] # initial condition
    )

    # must-run constraint
    @constraint(dispatch_problem, must_run[g in generators, t in periods],
        commit[g, t] >= generators_data[g]["must_run"]
    )

    # cost structure
    @constraint(dispatch_problem, [g in generators, t in periods],
    startup[g, t] == sum(delta[g, s, t] for s in startup_categories[g])  # the startup must be of one category
    )

    @constraint(dispatch_problem, [g in generators, t in periods, s in startup_categories[g]; s < last(startup_categories[g]) && t >= generators_data[g]["startup"][s+1]["lag"]],
    delta[g, s, t] <= sum(shutdown[g, t2] for t2 in periods if (t2 <= t-generators_data[g]["startup"][s]["lag"]) && (t2 >= t-generators_data[g]["startup"][s+1]["lag"]+1)) # find the right startup category
    )

    @constraint(dispatch_problem, [g in generators],  # initial condition on the startup category
    0 == sum(
        sum(delta[g, s, t] for t in periods if t>=max(1, generators_data[g]["startup"][s+1]["lag"] - generators_data[g]["time_down_t0"] + 1)
        && t<=min(generators_data[g]["startup"][s+1]["lag"] - 1, maximum(periods)))
        for s in startup_categories[g] if s < last(startup_categories[g]))
    )

    @constraint(dispatch_problem, [g in generators, t in periods],
    prod[g, t] == sum(pwl_prod[g, l, t] * (generators_data[g]["piecewise_production"][l]["mw"] - generators_data[g]["piecewise_production"][1]["mw"]) for l in pwl_points[g])
    )

    @constraint(dispatch_problem, [g in generators, t in periods],
    cost[g, t] == sum(pwl_prod[g, l, t] * (generators_data[g]["piecewise_production"][l]["cost"] - generators_data[g]["piecewise_production"][1]["cost"]) for l in pwl_points[g])
    )

    @constraint(dispatch_problem, [g in generators, t in periods],
    commit[g, t] == sum(pwl_prod[g, l, t] for l in pwl_points[g])
    )

    return dispatch_problem
end

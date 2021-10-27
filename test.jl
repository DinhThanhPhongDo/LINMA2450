import GLPK,JuMP, JSON
using GLPK,JuMP, JSON


dict = JSON.parsefile("small.json")
c = dict["utility"]
a = dict["weight"]
N = dict["N"][end]
b = dict["b"]

function SolverModel(c,a,N,b)

    model = Model(GLPK.Optimizer)
    @variable(model, x[1:N], Int)
    @show sum(c[i]*x[i] for i in 1:N)

    @objective(model, Max, sum(c[i]*x[i] for i in 1:N))

    @constraint(model, sum(a[i]*x[i] for i in 1:N)<= b)
    @constraint(model, con[i=1:N],x[i] >= 0)

    print(model)
    optimize!(model)

    #@show solution_summary(model, verbose=true)
    return model

    
function greedy(c,a,N,b)

    x = { i:0 for i in 1:N}
    return x
#import GLPK,JuMP, JSON, Base, Gurobi, ProgressBars
using GLPK,JuMP, JSON, Base, Gurobi, ProgressBars

function SolverModel(c,a,N,b)
    print("==============================Gurobi/ GLPK================================")
    model = Model(Gurobi.Optimizer)
    
    #turn off some settings
    set_optimizer_attribute(model, "Presolve", 0)
    set_optimizer_attribute(model, "Heuristics", 0)
    set_optimizer_attribute(model, "Cuts", 0)

    #set limit time
    set_time_limit_sec(model, 20.0)

    #the model itself
    @variable(model, x[1:N], Int)

    @objective(model, Max, sum(c[i]*x[i] for i in 1:N))

    @constraint(model, sum(a[i]*x[i] for i in 1:N)<= b)
    @constraint(model, con[i=1:N],x[i] >= 0)

    optimize!(model)
    return model
end


function greedy(c,a,N,b)
    println("==============================GreedyAlgorithm==============================")

    utility = c ./ a #division terme par terme

    list     = [ (i, utility[i]) for i in 1:N]

    Base.sort!(list, by = x -> x[2], rev = true) 

    x = Dict(i => 0 for i in 1:N)
    
    current_weight = 0
    for (i,u) in list
        if current_weight + a[i]< b
            x[i] = floor((b-current_weight)/a[i],digits = 0)
            current_weight += x[i]* a[i]
        end
    end
    
    objective_value = sum(c[i]*x[i] for i in 1:N)
    x = sort(collect(x), by=x->x[2], rev = true)
    return objective_value, x
end
function DynamicProgramming(c,a,N,b)
    println("==============================DynamicProgramming==============================")

    b = convert(UInt16,b)
    table = zeros(Int64, N+1, b+1) 
    dict = Dict(i => (c[i],a[i]) for i in 1:N)
    for i in ProgressBar(1:N+1)

        for w in 1:b+1
            if (i == 1 || w == 1) #initialisation
                table[i,w] = 0
            else
                tmax=floor(w/dict[i-1][2], digits=0)
                tmax=convert(UInt16,tmax)
                
                totest = [ t *dict[i-1][1] + table[i-1, w - t*dict[i-1][2]] for t in 0:tmax if t*dict[i-1][2] <= w-1]
                table[i,w] = findmax(totest)[1]
            end
        end
    end

    #get x
    x = Dict(i => 0 for i in 1:N)
    w = b+1
    for i in (N+1:-1:2)
        if table[i,w] == table[i-1,w]
            x[i-1] = 0
            continue
        else
            tmax=(w-1)/dict[i-1][2]
            tmax=floor(tmax, digits=0)
            tmax=convert(UInt16,tmax)  
            flag = false
            for t in (tmax:-1:1)
                if (table[i,w]- table[i-1, w - t*dict[i-1][2]]) % dict[i-1][1] == 0  
                    flag = true
                    w = w-t*dict[i-1][2]
                    x[i-1] = t
                    break
                end
            end
        end
    end
    x = sort(collect(x), by=x->x[2], rev = true)
    return table[N+1,b+1],x
end
###############################################################################################################################################

# Parsing : extracting datas from .json files 
dict = JSON.parsefile("large2.json")

c = dict["utility"]
a = dict["weight"]
N = dict["N"][end]
b = dict["b"]

#Solving the problem with Gurobi Solver:

@time model = SolverModel(c,a,N,b)
@show solution_summary(model,verbose=true)
@show objective_value(model)

#Solving the problem with GreedyAlgorithm

@time aTx,x=greedy(c,a,N,b)
print("objective value: ")
println(aTx)
print("Variables: ")
println(x)

#Solving the problem with DynamicProgramming

@time aTx,x = DynamicProgramming(c,a,N,b)
print("objective value: ")
println(aTx)
print("Variables: ")
println(x)

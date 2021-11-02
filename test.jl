import GLPK,JuMP, JSON, Base, Gurobi
using GLPK,JuMP, JSON, Base, Gurobi


dict = JSON.parsefile("small.json")
c = dict["utility"]
a = dict["weight"]
N = dict["N"][end]
b = dict["b"]

function SolverModel(c,a,N,b)
    print("==============================Gurobi/ GLPK================================")
    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model, "Presolve", 0)
    set_optimizer_attribute(model, "Heuristics", 0)
    set_optimizer_attribute(model, "Cuts", 0)
    @variable(model, x[1:N], Int)
    @show sum(c[i]*x[i] for i in 1:N)

    @objective(model, Max, sum(c[i]*x[i] for i in 1:N))

    @constraint(model, sum(a[i]*x[i] for i in 1:N)<= b)
    @constraint(model, con[i=1:N],x[i] >= 0)

    optimize!(model)

    @show solution_summary(model, verbose=true)
    return model
end


function greedy(c,a,N,b)
    println("==============================GreedyAlgorithm==============================")

    utility = c ./ a #division terme par terme

    list     = [ (i, utility[i]) for i in 1:N]
    dict     = Dict( i=> c[i]/a[i] for i in 1:N)
    Base.sort!(list, by = x -> x[2], rev = true) #silent function applied on list

    x = Dict(i => 0 for i in 1:N)
    
    previous_weight = -1.0
    current_weight = 0.0

    while (current_weight != previous_weight)
        previous_weight = current_weight

        for (i,u) in list

            while current_weight+a[i] <b
                x[i] += 1.0
                current_weight += a[i]
            end
        end
    
    end
    
    objective_value = sum(c[i]*x[i] for i in 1:N)
    x = sort(collect(x), by=x->x[2], rev = true)
    return objective_value, current_weight,x
end
function DynamicProgramming2(c,a,N,b)
    println("==============================DynamicProgramming==============================")

    b = convert(UInt16,b)
    table = zeros(Int64, N+1, b+1) 
    dict = Dict(i => (c[i],a[i]) for i in 1:N)
    for i in 1:N+1
        for w in 1:b+1
            if (i == 1 || w == 1) #initialisation
                table[i,w] = 0
            else
                tmax=b/dict[i-1][2]
                tmax=floor(tmax, digits=0)
                tmax=convert(UInt16,tmax)
                for t in 0:tmax
                    flag =false
                    if t*dict[i-1][2] <= w-1
                        flag =true
                        table[i,w] = max(table[i-1, w],
                                        t *dict[i-1][1] + table[i-1, w - t*dict[i-1][2]],
                                        table[i,w])
                    end
                    if flag == false
                        break
                    end
                end                
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

@time model = SolverModel(c,a,N,b)
@show objective_value(model)

@time aa,bb,cc=greedy(c,a,N,b)
println(aa,cc)

@time println(DynamicProgramming2(c,a,N,b))
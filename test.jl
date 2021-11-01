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

    #print(model)
    optimize!(model)

    @show solution_summary(model, verbose=true)
    return model
end
@time SolverModel(c,a,N,b)

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
    x = sort(collect(x), by=x->x[2])
    return objective_value, current_weight,x
end
@time greedy(c,a,N,b)
function DynamicProgramming(c,a,N,b)
    println("==============================DynamicProgramming==============================")

    b = convert(UInt32,b)
    table = zeros(Int64, N+1, b+1)
    dict = Dict(i => (c[i],a[i]) for i in 1:N)

    for i in 1:N+1

        for w in 1:b+1

            if (i == 1 || w == 1) #initialiser
                table[i,w] = 0

            #tant que le poids n'est pas suffisant
            elseif dict[i-1][2] <= w-1
                table[i,w] = max(table[i-1, w],
                                    dict[i-1][1] + table[i-1, w - dict[i-1][2]])
            else dict[i-1][2] < w-1

                table[i,w] = table[i-1,w]
            end
        end
    end
    return table[N+1,b+1]
end

function DynamicProgramming2(c,a,N,b)

    println("==============================DynamicProgramming==============================")
    b = convert(UInt16,b)
    table = zeros(Int64, N+1, b+1) 
    dict = Dict(i => (c[i],a[i]) for i in 1:N)#c=profit,a=poids
    for i in 1:N+1
        for w in 1:b+1
            if (i == 1 || w == 1) #initialisation
                table[i,w] = 0
            else
                tmax=b/dict[i-1][2]
                tmax=floor(tmax, digits=0)
                tmax=convert(UInt16,tmax)
                for t in 0:tmax
                    #Im not too sure how to create properly flag, but this one is to avoid to cycle too much in the "t loop"
                    flag =false
                    #tant que le poids n'est pas suffisant
                    if t*dict[i-1][2] <= w-1
                        flag =true
                        #dict[i-1] acces the current item. table[i-1] acces the previous item! Sorry but Julia index begin. 
                        #Si tu trouves une façon de rendre les indexs plus lisible, tu peux toujours modifier
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
            # t to modify according to textbook p42 and not from 0 to 1000
            for t in 0:1000#dict[i][2]/b
                #Im not too sure how to create properly flag, but this one is to avoid to cycle too much in the "t loop"
                flag =false
                if (i == 1 || w == 1) #initialisation
                    table[i,w] = 0
                #tant que le poids n'est pas suffisant
                elseif t*dict[i-1][2] <= w-1
                    println(b/dict[i][2])
                    flag =true
                    #dict[i-1] acces the current item. table[i-1] acces the previous item! Sorry but Julia index begin. 
                    #Si tu trouves une façon de rendre les indexs plus lisible, tu peux toujours modifier
                    table[i,w] = max(table[i-1, w],
                                    t *dict[i-1][1] + table[i-1, w - t*dict[i-1][2]],
                                    table[i,w])
                end
    #get x
    x = Dict(i => 0 for i in 1:N)
    w = b+1
    for i in 1:N+1
        #println(i-1)
        #println(table[i,:])
    end
    for i in (N+1:-1:2)
            #println(i-1)
        if table[i,w] == table[i-1,w]
            x[i-1] = 0
            #println((i-1,x[i-1]))
            continue
        else
            tmax=(w-1)/dict[i-1][2]
            tmax=floor(tmax, digits=0)
            tmax=convert(UInt16,tmax)  
            #print("tmax =")    
            #println(tmax)
            #print("actual weight= ")
            #println(w)
            #print("first")
            #println(i-1)
            flag = false
            for t in (tmax:-1:1)
                println(t)
                if (table[i,w]- table[i-1, w - t*dict[i-1][2]]) % dict[i-1][1] == 0  
                    flag = true
                    w = w-t*dict[i-1][2]
                    #print("weight=")
                    #println(w)
                    x[i-1] = t
                    #println((i-1,x[i-1]))
                    break
                end
            end
            #if flag == false
            #    println("there is a problem")
            #    println(i-1)
            #end
        end
    end
    #print("my sum is:")
    #println(sum(c[i]*x[i] for i in 1:N))
     x = sort(collect(x), by=x->x[2], rev = true)
    return table[N+1,b+1],x
end



model = SolverModel(c,a,N,b)
@show objective_value(model)


println(greedy(c,a,N,b)[1],greedy(c,a,N,b)[3])
println(greedy(c,a,N,b))

#for the medium problem, we can obtain the same results as the GLPK solver. Very slow for large problem. 
#there exist also a DynamicProgramming solver, but this one is the one from the github and is only for the 0-1 knapsack problem i think
println(DynamicProgramming2(c,a,N,b))




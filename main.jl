import GLPK, JuMP
using GLPK,JuMP


model = Model(GLPK.Optimizer)
@variable(model, x, Int)
@variable(model, 0 <= y <= 3)

@objective(model, Min, 12x + 20y)


@constraint(model, c1, 6x + 8y >= 100)
@constraint(model, c2, 7x + 12y >= 120)

print(model)
optimize!(model)
@show termination_status(model)
@show primal_status(model)
@show dual_status(model)
@show objective_value(model)
@show value(x)
@show value(y)
@show shadow_price(c1)
@show shadow_price(c2)

function DynamicProgramming(c,a,N,b)
    println("==============================DynamicProgramming==============================")
    b = convert(UInt16,b)
    table = zeros(Int64, N+1, b+1) 
    dict = Dict(i => (c[i],a[i]) for i in 1:N)
    for i in 1:N+1

        for w in 1:b+1

            for t in 0:10

                if (i == 1 || w == 1) #initialiser
                    table[i,w] = 0

                #tant que le poids n'est pas suffisant
                elseif dict[i-1][2] <= w-1 - t*dict[i][2]

                        table[i,w] = max(table[i-1, w],
                                        t *dict[i-1][1] + table[i-1, w - dict[i-1][2]])
                else dict[i-1][2] < w-1

                    table[i,w] = table[i-1,w]
                end
            end
        end
    end
    return table[101,:]
end
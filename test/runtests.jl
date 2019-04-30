using Test
using GraphSearchLight

macro testblock(ex)
    quote
        try
            $(esc(eval(ex)))
            true
        catch err
            isa(err, ErrorException) ? false : rethrow(err)
        end
    end
end

include("test_search.jl")

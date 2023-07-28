using HybridAStar
using Test

@testset "Dubin Car" begin
    include("../examples/demo_dubin_car.jl")
    @test path == []
end

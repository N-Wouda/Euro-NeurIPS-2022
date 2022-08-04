cmake_minimum_required(VERSION 3.2)

set(geneticSources
        GeneticAlgorithm.cpp
        crossover/orderedExchange.cpp
        crossover/selectiveRouteExchange.cpp)

set(individualSources
        Individual.cpp)

set(localSearchSources
        LocalSearch.cpp
        operators/operators.cpp
        operators/moveSingleClient.cpp
        operators/moveTwoClients.cpp
        operators/moveTwoClientsReversed.cpp
        operators/swapTwoClientPairs.cpp
        operators/swapTwoClientsForOne.cpp
        operators/swapTwoSingleClients.cpp
        operators/twoOptBetweenTrips.cpp
        operators/twoOptWithinTrip.cpp
        operators/relocateStar.cpp)

set(paramsSources
        Params.cpp)

set(populationSources
        Population.cpp)

set(resultSources
        Result.cpp)

set(statisticsSources
        Statistics.cpp)

set(timeWindowSegmentSources
        TimeWindowSegment.cpp)

# (Internal) library
add_library(hgs
        ${geneticSources}
        ${individualSources}
        ${localSearchSources}
        ${paramsSources}
        ${populationSources}
        ${resultSources}
        ${statisticsSources}
        ${timeWindowSegmentSources})

# Executable
add_executable(genvrp main.cpp)
target_link_libraries(genvrp PRIVATE hgs)

# Python bindings
pybind11_add_module(hgspy bindings.cpp)
target_link_libraries(hgspy PRIVATE hgs)

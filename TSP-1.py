"""Simple Travelling Salesperson Problem (TSP) between States."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    data = {}
    data['distance_matrix'] = [
        [0, 130, 192, 317, 1410, 2003, 1640, 1811],
        [130, 0, 433, 297, 1523, 2120, 1720, 1908],
        [347, 433, 0, 273, 1105, 1711, 1298, 1475],
        [304, 297, 273, 0, 1357, 1970, 1431, 1695],
        [1410, 1523, 1105, 1357, 0, 686, 1267, 782],
        [2003, 2120, 1711, 1970, 686, 0, 1342, 869],
        [1640, 1720, 1298, 1431, 1267, 1342, 0, 955],
        [1811, 1908, 1475, 1695, 782, 869, 955, 0],
    ]  # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

my_dict = { 0 : 'Tamil Nadu' ,
                1 : 'Kerala',
                2 : 'Andhra Pradhesh',
                3 : 'Karnataka',
                4 : 'Assam',
                5 : 'Manipur',
                6 : 'Uttar Pradhesh',
                7 : 'Sikkim',
            }

def print_solution(my_dict, manager, routing, solution):
    print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(my_dict.get(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(my_dict.get(0))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)

def main():
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(my_dict, manager, routing, solution)

if __name__ == '__main__':
    main()
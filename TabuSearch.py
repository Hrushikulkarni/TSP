import time
def calculate_total_distance(route, distance_matrix):
    total_distance = 0
    for i in range(len(route)):
        total_distance += distance_matrix[route[i-1]][route[i]]
    return total_distance

def nearest_neighbor_heuristic(distance_matrix, start=0):
    unvisited = list(range(len(distance_matrix)))
    route = [start]
    unvisited.remove(start)

    while unvisited:
        current = route[-1]
        nearest = min(unvisited, key=lambda x: distance_matrix[current][x])
        route.append(nearest)
        unvisited.remove(nearest)

    return route

def generate_neighbors(route):
    neighbors = []
    for i in range(1, len(route) - 2):
        for j in range(i + 1, len(route)):
            neighbor = route[:]
            neighbor[i:j] = neighbor[j-1:i-1:-1]
            neighbors.append(neighbor)
    return neighbors


def tabu_search(distance_matrix, iterations=100, tabu_list_size=10):
    current_solution = nearest_neighbor_heuristic(distance_matrix)
    best_solution = current_solution[:]
    best_cost = calculate_total_distance(best_solution, distance_matrix)
    
    tabu_list = []

    for _ in range(iterations):
        neighbors = generate_neighbors(current_solution)
        neighbors = [neighbor for neighbor in neighbors if neighbor not in tabu_list]
        
        best_neighbor = min(neighbors, key=lambda x: calculate_total_distance(x, distance_matrix))
        best_neighbor_cost = calculate_total_distance(best_neighbor, distance_matrix)

        if best_neighbor_cost < best_cost:
            best_solution = best_neighbor[:]
            best_cost = best_neighbor_cost
            tabu_list.append(best_neighbor)
            if len(tabu_list) > tabu_list_size:
                tabu_list.pop(0)
        
        current_solution = best_neighbor

    return best_solution, best_cost

def main():
    input = open('input.txt', "r").read()
    inputSize = input.split('\n')[0]
    distance_matrix = [item.split() for item in input.split('\n')[1:]]
    distance_matrix = [[column for column in row] for row in distance_matrix]
    distance_matrix = [list(map(float,i)) for i in distance_matrix]
    seconds_1 = time.time()
    best_route, best_distance = tabu_search(distance_matrix, iterations=1000, tabu_list_size=100)
    seconds_2 = time.time()
    best_route.append(best_route[0])
    print("Best Route:", best_route)
    print("Total Distance:", best_distance)
    print("Time taken : ", seconds_2 - seconds_1)

if __name__ == "__main__":
    main()
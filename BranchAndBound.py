import math
import time
from copy import copy
class Node:
    def __init__(self, vertex, curr_cost, lower_bound, visited, path):
        self.vertex=vertex
        self.curr_cost=curr_cost
        self.lower_bound=lower_bound
        self.visited=visited
        self.path=path

maxsize = float('inf')

def initialize_Heuristic(adj, current_bound, num_vertices):
    for i in range(num_vertices):
        current_bound += (first_Min(adj, i) +
                       second_Min(adj, i))
    current_bound = math.ceil(current_bound / 2)
    return current_bound

def update_Heuristics(adj, vertex_1, vertex_2, current_bound, start):
    if vertex_1 == start:
        current_bound -= ((first_Min(adj, vertex_1) +
                        first_Min(adj, vertex_2)) / 2)
    else:
        current_bound -= ((second_Min(adj, vertex_1) +
                        first_Min(adj, vertex_2)) / 2)
    return current_bound


def first_Min(adj, i):
    min = maxsize
    for k in range(len(adj)):
        if adj[i][k] < min and i != k:
            min = adj[i][k]
    return min


def second_Min(adj, i):
    first, second = maxsize, maxsize
    for j in range(len(adj)):
        if i != j:
            current_element = adj[i][j]

            if current_element <= first:
                second = first
                first = current_element
            elif current_element <= second and current_element != first:
                second = current_element

    return second

def BNB_TSP(adj,start):
    stack = []
    path = [start]
    visited = [False] * len(adj)
    current_bound = 0
    num_vertices = len(adj)
    current_bound = initialize_Heuristic(adj, current_bound, num_vertices)
    tsp_cost = math.inf
    best_path = None
    visited[start] = True
    stack.append(Node(start, 0, current_bound, visited, path))

    while len(stack)!=0:
        pop_v = stack.pop()
        curr_cost = copy(pop_v.curr_cost)
        for n in range(num_vertices):
            visited_par = copy(pop_v.visited)
            if all(visited_par):
                if pop_v.curr_cost + adj[pop_v.path[-1]][pop_v.path[0]] < tsp_cost:
                    tsp_cost = pop_v.curr_cost + adj[pop_v.path[-1]][pop_v.path[0]]
                    best_path = pop_v.path
                    break

            if not visited_par[n]:
                visited_par[n] = True
                path = copy(pop_v.path)
                current_bound = copy(pop_v.lower_bound)

                if current_bound + adj[pop_v.vertex][n] > tsp_cost:
                    continue

                current_bound = update_Heuristics(adj, path[-1], n, current_bound, start)
                path.append(n)
                stack.append(Node(n, curr_cost+adj[pop_v.vertex][n], current_bound, visited_par, path))

    best_path.append(best_path[0])
    return best_path, tsp_cost

def main():
    input = open('input.txt', "r").read()
    inputSize = input.split('\n')[0]
    inputMatrix = [item.split() for item in input.split('\n')[1:]]
    inputMatrix = [[column for column in row] for row in inputMatrix]
    inputMatrix = [list(map(float,i)) for i in inputMatrix]
    seconds_1 = time.time()
    best_path, tsp_cost = BNB_TSP(inputMatrix, 0)
    seconds_2 = time.time()
    print("Best Path will be: ", best_path)
    print("TSP Cost will be: ", tsp_cost)
    print("Time taken : ", seconds_2 - seconds_1)


if __name__ == "__main__":
    main()

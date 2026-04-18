#pragma once
#include <vector>
#include "Map.h"
#include <unordered_map>
 
// clase con los algoritmos de busqueda
class Search{
 
public:
    // reconstruye el camino desde el goal hasta el start
    static std::vector<std::pair<int,int>> reconstruct(
        const std::unordered_map<std::pair<int,int>, std::pair<int,int>>& pathCache,
        const std::pair<int,int>& goal
    );
 
    // calcula la distancia manhattan entre dos puntos
    static float Heuristic(
        std::pair<int,int> actual,
        std::pair<int,int> goal
    );
 
    // busqueda BFS, garantiza el camino mas corto
    static std::vector<std::pair<int,int>> BFS(
        const Map& map,
        std::pair<int,int> start,
        std::pair<int,int> goal
    );
 
    // busqueda greedy, va siempre hacia donde la heuristica es menor
    static std::vector<std::pair<int,int>> Greedy(
        const Map& map,
        std::pair<int,int> start,
        std::pair<int,int> goal
    );
 
    // busqueda A*, combina costo real y heuristica (f = g + h)
    static std::vector<std::pair<int,int>> Astar(
        const Map& map,
        std::pair<int,int> start,
        std::pair<int,int> goal
    );
};
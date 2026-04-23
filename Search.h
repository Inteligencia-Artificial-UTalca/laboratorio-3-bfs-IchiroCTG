#pragma once
#include <vector>
#include "Map.h"
#include <unordered_map>
 
using namespace std;
// clase con los algoritmos de busqueda

// Atributos:
/*      
        const Map& map, // Mapa a buscar
        pair<int,int> start, // Punto de inicio
        pair<int,int> goal, // Punto objetivo

*/

// Uso de estatico para llamar a Search sin la necesidad de crear una instancia, por ejemplo: Search::BFS(map, start, goal);

class Search{
 
public:

    // reconstruye el camino desde el goal hasta el start
    static vector<pair<int,int>> reconstruct(
        const unordered_map<pair<int,int>, pair<int,int>>& pathCache,
        const pair<int,int>& goal
    );
 
    // calcula la distancia manhattan entre dos puntos
    static float Heuristic(
        pair<int,int> actual,
        pair<int,int> goal
    );
 
    // busqueda BFS, garantiza el camino mas corto
    static vector<pair<int,int>> BFS(
        const Map& map,
        std::pair<int,int> start,
        std::pair<int,int> goal
    );
 
    // busqueda greedy, va siempre hacia donde la heuristica es menor
    static vector<pair<int,int>> Greedy(
        const Map& map,
        pair<int,int> start,
        pair<int,int> goal
    );
 
    // busqueda A*, combina costo real y heuristica (f = g + h)
    static vector<pair<int,int>> Astar(
        const Map& map,
        pair<int,int> start,
        pair<int,int> goal,
        bool esAltura
    );

    static vector<pair<int,int>> WAstar(
        const Map& map,
        pair<int,int> start,
        pair<int,int> goal,
        float weight,// Peso para la heuristica, mayor a 1 hace que el algoritmo sea más agresivo hacia el objetivo
        bool esAltura 
    );

    static float calcularLargoCamino(
        const Map& map,
        const std::vector<std::pair<int,int>>& camino,
        bool esAltura
    );
};
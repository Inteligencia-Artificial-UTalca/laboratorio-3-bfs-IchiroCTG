#include "Search.h"
#include <queue>
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <cmath>
#include <functional>

const float CDiagonal = 1.41; // Costo de movimiento diagonal (aproximadamente sqrt(2))
const float COrtogonales = 1.0; // Costo de movimiento ortogonales
const float Umbral = 4; // Diferencia maxima de altura antes de que bajar sea peligroso

namespace std
{
    // Necesitamos esto para usar pair<int,int> como clave en unordered_map
    template<> struct hash<pair<int,int>>
    {
        size_t operator()(const pair<int,int>& p) const noexcept
        {
            hash<int> hasher;
            return hasher(p.first) ^ (hasher(p.second) << 1);
        }
    };
}

// ─────────────────────────────────────────────
//  reconstruct
//  Viaje desde el goal hasta el start usando pathCache, 
//  luego invierte el resultado para obtener el camino desde el start hasta el goal.
// ─────────────────────────────────────────────

pair <int,int> dirs[]{ // arreglo de direcciones para moverse en el grid 
    {-1,0}, // Arriba
    {-1,1}, // Arriba-Derecha
    {0,1},  // Derecha
    {1,1},  // Abajo-Derecha
    {1,0},  // Abajo
    {1,-1}, // Abajo-Izquierda
    {0,-1}, // Izquierda
    {-1,-1} // Arriba-Izquierda
}; 

// Se calcula el costo de moverse entre dos celdas, considerando la diferencia de altura y el umbral de seguridad.
// Si sube cuesta más, si baja cuesta menos, pero si baja más que el umbral, se considera peligroso y se asigna un costo muy alto para evitar ese camino.
float CalcularCostoMovimiento(float alturaActual, float alturaSiguiente, bool esDiagonal, bool esAltura){
    float costoBase=0.0f;
    if(esDiagonal){
        costoBase = CDiagonal;
         // Costo base para movimiento diagonal
    }

    else{
        // Costo base para movimiento ortogonal
        costoBase = COrtogonales; 
    }

    // Si no se considera la altura, el costo es el base sin modificaciones retorna
    if(!esAltura) return costoBase; 
    
    // Diferencia de altura entre la celda actual y la siguiente
    float diferenciaAltura = alturaSiguiente - alturaActual;  // postivo si sube, negativo si baja

    if(diferenciaAltura>0){
        // si sube, el costo aumenta proporcionalmente a la diferencia de altura
        return costoBase + diferenciaAltura; // Costo base + penalización por subir
    }
    else{
        float absDiferencia = abs(diferenciaAltura);
        if(absDiferencia >= Umbral){
            // muy empinado, peligroso bajar
            return costoBase + (absDiferencia- Umbral);
        }
        else{
            // bajada normal, cuesta menos que pero min 0.1 para evitar que bajar sea gratis
            // Costo base - bonificación por bajar, pero no menos que el costo ortogonal mínimo
            return max(costoBase - absDiferencia, 0.1f); 
        }
    }
}

vector<pair<int,int>> Search::reconstruct(
    const unordered_map<pair<int,int>,pair<int,int>>& pathCache,
    const pair<int,int>& goal)
{
    deque<pair<int,int>> path;
    auto node = goal; // goal es el nodo final del camino

    while(true){
        path.push_front(node); // path se agregan al frente para evitar invertir al final
        auto it = pathCache.find(node);  // Asignacion automatica del iterador al resultado de find
        if(it == pathCache.end()) break; // llegamos al nodo inicial, que no tiene padre en el cache
        node = it->second;               // Se mueve al padre del nodo actual
    }

    vector<pair<int,int>> vec(path.begin(), path.end());
    return vec;
}

// ─────────────────────────────────────────────
//  Heuristic  (Distancia Octile)  D * (dx + dy) + (D2 - 2*D) * min(dx, dy) con D=1 y D2=sqrt(2) para movimientos diagonales
//  Estimación del costo restante desde 'actual' hasta 'goal', considerando movimientos ortogonales y diagonales en un grid.
// ─────────────────────────────────────────────
float Search::Heuristic(pair<int,int> current, pair<int,int> goal)
{ 
    float dx = abs(current.first - goal.first);
    float dy = abs(current.second - goal.second);

    //mover en diagonal cuesta sqrt(2) y mover en ortogonal cuesta 1, por eso se usa esa formula
    return COrtogonales * (dx + dy) + (CDiagonal - 2.0f * COrtogonales) * min(dx, dy);
}

// ─────────────────────────────────────────────
//  BFS
//  Garantiza el camino más corto en un grid sin pesos, pero puede ser lento en mapas grandes o con muchos obstáculos.
// ─────────────────────────────────────────────
vector<pair<int,int>> Search::BFS(
    const Map& map,
    pair<int,int> start,
    pair<int,int> goal)
{
    cout << "===========================\nEjecutando BFS...\n";
    auto startTime = chrono::high_resolution_clock::now();


    std::vector<std::vector<bool>> visited(map.h, std::vector<bool>(map.w, false));
    queue<pair<int,int>> OPEN;
    unordered_map<pair<int,int>,pair<int,int>> pathCache; // Hijo -> Padre

    OPEN.push(start);
    visited[start.first][start.second] = true;

    while(!OPEN.empty()){
        auto pos = OPEN.front();
        OPEN.pop();

        if(pos == goal){ // Si encontramos el objetivo, reconstruimos el camino y terminamos
            auto endTime = chrono::high_resolution_clock::now();
            int count = 0;
            for(int i = 0; i < map.h; i++)
                for(int j = 0; j < map.w; j++)
                    if(visited[i][j]) count++;
            cout << "NODOS VISITADOS: " << count << endl;
            cout << "NODOS ABIERTOS: "    << OPEN.size() << endl;
            cout << "Encontrado en " << (endTime-startTime).count()/1000000.0 << "ms\n";
            return reconstruct(pathCache, pos);
        }

        for(auto dir : dirs){
            pair<int,int> next = {pos.first + dir.first, pos.second + dir.second};

            // Bounds check
            if(next.first < 0 || next.first >= map.h ||
               next.second < 0 || next.second >= map.w)
                continue;

            // Wall or already visited
            //if(map._map[next.first][next.second] == 1) continue;
            if(visited[next.first][next.second])        continue;

            visited[next.first][next.second] = true;
            pathCache[next] = pos;
            OPEN.push(next);
        }
    }

    cout << "!Camino no encontrado!\n";
    vector<pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}

// ─────────────────────────────────────────────
//  Greedy Best-First Search  (Lab 4)
//  Expande el nodo que parece estar más cerca del objetivo según la heurística, 
//  pero no garantiza el camino más corto y puede quedar atrapado en callejones sin salida.
// ─────────────────────────────────────────────
vector<pair<int,int>> Search::Greedy(
    const Map& map,
    pair<int,int> start,
    pair<int,int> goal)
{
    cout << "===========================\nEjecutando Greedy BFS...\n";
    auto startTime = chrono::high_resolution_clock::now();

    // Cola de Prioridad: min-heap por h(n)
    // Elemento: { h_valor, posicion }
    using Node = pair<float, pair<int,int>>;
    priority_queue<Node, vector<Node>, greater<Node>> OPEN;

    unordered_map<pair<int,int>, pair<int,int>> pathCache; // hijo -> padre
    unordered_map<pair<int,int>, bool> visited;

    OPEN.push({Heuristic(start, goal), start});
    visited[start] = true;

    while(!OPEN.empty()){ // Mientras haya nodos por expandir
        Node top = OPEN.top();
        OPEN.pop();
        pair<int,int> pos = top.second;

        if(pos == goal){ // Si encontramos el objetivo, reconstruimos el camino y terminamos
            auto endTime = chrono::high_resolution_clock::now(); // tiempo de finalización
            cout << "NODOS VISITADOS: " << visited.size() << endl; // cantidad de nodos visitados
            cout << "NODOS ABIERTOS: "    << OPEN.size() << endl; // cantidad de nodos aún por expandir
            cout << "Encontrado en " << (endTime-startTime).count()/1000000.0 << "ms\n"; // tiempo total de ejecución en milisegundos
            return reconstruct(pathCache, pos);
        }

        for(auto dir : dirs){ // Para cada vecino de pos (arriba, derecha, abajo, izquierda)
            pair<int,int> next = {pos.first + dir.first, pos.second + dir.second};

            if(next.first < 0 || next.first >= map.h ||
               next.second < 0 || next.second >= map.w)
                continue;

            //if(map._map[next.first][next.second] == 1) continue;
            if(visited.count(next))                     continue;

            visited[next] = true;
            pathCache[next] = pos;
            OPEN.push({Heuristic(next, goal), next});
        }
    }

    cout << "!Camino no encontrado!\n";
    vector<pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}

// ─────────────────────────────────────────────
//  A*  (Lab 5)
//  Expande el nodo con el menor f = g + h,
//  donde g es el costo real desde el inicio hasta el nodo, 
//  y h es la heurística (estimación del costo restante hasta el objetivo).
// ─────────────────────────────────────────────
vector<pair<int,int>> Search::Astar(
    const Map& map,
    pair<int,int> start,
    pair<int,int> goal,
    bool esAltura)
{
    cout << "===========================\nEjecutando A*...\n";
    auto startTime = chrono::high_resolution_clock::now();


    // Priority queue: min-heap by f = g + h
    using Node = pair<float, pair<int,int>>;
    priority_queue<Node, vector<Node>, greater<Node>> OPEN;

    unordered_map<pair<int,int>, pair<int,int>> pathCache; // hijo -> padre
    unordered_map<pair<int,int>, float> gCost;                  // costo real desde el inicio hasta este nodo
    unordered_map<pair<int,int>, bool>  closed;                 // cerrar nodos ya expandidos

    gCost[start] = 0.0f;
    OPEN.push({Heuristic(start, goal), start});

    while(!OPEN.empty()){ // Mientras haya nodos por expandir
        Node top = OPEN.top();
        OPEN.pop();
        pair<int,int> pos = top.second;

        if(closed.count(pos)) continue; // Expandir los nodos más baratos primero, pero si ya fue cerrado, lo ignoramos
        closed[pos] = true;

        if(pos == goal){
            auto endTime = chrono::high_resolution_clock::now();
            cout << "NODOS VISITADOS: " << closed.size() << endl;
            cout << "NODOS ABIERTOS: "    << OPEN.size() << endl;
            cout << "Encontrado en " << (endTime-startTime).count()/1000000.0 << "ms\n";
            return reconstruct(pathCache, pos);
        }

        for(auto dir : dirs){ // Para cada vecino de pos (arriba, derecha, abajo, izquierda)
            pair<int,int> next = {pos.first + dir.first, pos.second + dir.second};

            if(next.first < 0 || next.first >= map.h ||
               next.second < 0 || next.second >= map.w)
                continue;

            if(!esAltura && map._map[next.first][next.second] == 1) continue;
            if(closed.count(next))                      continue;

            bool esDiagonal = (dir.first != 0 && dir.second != 0);
            
            float NuevoG = gCost[pos] + 
            CalcularCostoMovimiento (map._map[pos.first][pos.second], map._map[next.first][next.second], 
            esDiagonal, esAltura);
            
            if(!gCost.count(next) || NuevoG < gCost[next]){ // Costo de movimiento diagonal
                gCost[next] = NuevoG;
                pathCache[next] = pos;
                float f = NuevoG + Heuristic(next, goal);
                OPEN.push({f, next});

            }
            
        }
    }

    cout << "!Camino no encontrado!\n";
    vector<pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}

vector<pair<int,int>> Search::WAstar(
    const Map& map,
    pair<int,int> start,
    pair<int,int> goal, 
    float weight, 
    bool esAltura)
{

    cout << "===========================\nEjecutando Weighted A* (w=" << weight << ")...\n";
    auto startTime = chrono::high_resolution_clock::now(); // Tiempo de Inicio

    typedef pair<float, pair<int,int>> Node; // {f, posicion} creacion del tipo Node para la cola de prioridad
    
    priority_queue<Node, vector<Node>, greater<Node>> OPEN; // Cola de prioridad para nodos abiertos
    
    unordered_map<pair<int,int>, pair<int,int>> pathCache; // hijo -> padre
    unordered_map<pair<int,int>, float> gCost;                  // costo real desde el inicio hasta este nodo
    unordered_map<pair<int,int>, bool>  closed;                 // cerrar nodos ya expandidos

    gCost[start] = 0.0f; // El costo g del nodo inicial es 0
    OPEN.push({weight * Heuristic(start, goal), start}); // f = w *

    while (!OPEN.empty()) // Mientras haya nodos por expandir
    {
        Node top = OPEN.top();
        OPEN.pop();
        pair<int,int> pos = top.second;

        if (closed.count(pos)) continue; // Si el nodo ya fue cerrado, lo ignoramos
        closed[pos] = true; // Marcamos el nodo como cerrado

        if (pos == goal) // Si encontramos el objetivo, reconstruimos el camino y terminamos
        {
            auto endTime = chrono::high_resolution_clock::now(); // Tiempo de finalización
            cout << "NODOS VISITADOS: " << closed.size() << endl; // Cantidad de nodos visitados
            cout << "NODOS ABIERTOS: "    << OPEN.size() << endl; // Cantidad de nodos aún por expandir
            cout << "Encontrado en " << (endTime - startTime).count() / 1000000.0 << "ms\n"; // Tiempo total de ejecución en milisegundos
            return reconstruct(pathCache, pos); // Reconstrucción del camino desde el nodo objetivo hasta el nodo inicial
        }

        for(auto dir : dirs) // Para cada vecino de pos (arriba, derecha, abajo, izquierda)
        {
            pair<int,int> next = {pos.first + dir.first, pos.second + dir.second}; // Cálculo de la posición del vecino

            // Verificación de límites del mapa
            if (next.first < 0 || next.first >= map.h || next.second < 0 || next.second >= map.w)
                continue;

            if (!esAltura && map._map[next.first][next.second] == 1) continue; // Si el vecino es una pared, lo ignoramos
            if (closed.count(next)) continue; // Si el vecino ya fue cerrado, lo ignoramos
            bool esDiagonal = (dir.first != 0 && dir.second != 0); // Determinamos si el movimiento es diagonal
            float NCost = gCost[pos] + CalcularCostoMovimiento(
                                        map._map[pos.first][pos.second], 
                                        map._map[next.first][next.second],
                                        esDiagonal, esAltura); // Cálculo del costo g para el vecino
                                        
            // Solo agregamos a OPEN si encontramos un camino más barato hacia 'next'
            if (!gCost.count(next) || NCost < gCost[next])
            {
                gCost[next] = NCost; // Actualizamos el costo g para el vecino
                pathCache[next] = pos; // Actualizamos el padre del vecino en el cache de caminos
                float f = NCost + weight * Heuristic(next, goal); // Cálculo de f = g + w * h
                OPEN.push({f, next}); // Agregamos el vecino a la cola de prioridad
            }

        }
        
        
    }
    
    cout << "!Camino no encontrado!\n"; // Si se agotan los nodos por expandir sin encontrar el objetivo
    vector<pair<int,int>> path; // Retorno de un camino vacío (o con start y goal) para indicar que no se encontró un camino válido
    path.push_back(start);
    path.push_back(goal);
    return path;
}

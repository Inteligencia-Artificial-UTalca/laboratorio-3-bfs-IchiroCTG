#include "Map.h"
#include "Search.h"
#include "ColorMap.h"
#include <iostream>
#include <string>
#include <stdexcept>

// Returns true if the string represents a valid integer
bool isInteger(const std::string& s){
    if(s.empty()) return false;
    size_t start = (s[0] == '-') ? 1 : 0;
    for(size_t i = start; i < s.size(); i++)
        if(!std::isdigit(s[i])) return false;
    return true;
}

int main(int argc, char *argv[]){


    if(argc != 6){
        std::cerr << "Uso: " << argv[0]
                  << " <mapa.txt> <fila_inicio> <col_inicio> <fila_fin> <col_fin>\n";
        return 1;
    }


    for(int i = 2; i <= 5; i++){
        if(!isInteger(argv[i])){
            std::cerr << "Error: el argumento " << i
                      << " ('" << argv[i] << "') no es un entero valido.\n";
            return 1;
        }
    }

    std::string mapFile = argv[1];
    int x1 = std::atoi(argv[2]);
    int y1 = std::atoi(argv[3]);
    int x2 = std::atoi(argv[4]);
    int y2 = std::atoi(argv[5]);


    Map map(mapFile);
    if(map == Map()){         
        std::cerr << "Error: no se pudo cargar el mapa '" << mapFile << "'.\n";
        return 1;
    }

   
    ColorMap colorMap(map);

    // Print the raw map
    std::cout << "Mapa cargado:\n";
    colorMap.print();


    auto pathBFS = Search::BFS(map, {x1, y1}, {x2, y2});
    // Path length = number of steps (edges), which is nodes - 1
    std::cout << "Path length (BFS): " << (int)pathBFS.size() - 1 << "\n";
    colorMap.print(pathBFS);


    auto pathGreedy = Search::Greedy(map, {x1, y1}, {x2, y2});
    std::cout << "Path length (Greedy): " << (int)pathGreedy.size() - 1 << "\n";
    colorMap.print(pathGreedy);


    auto pathAstar = Search::Astar(map, {x1, y1}, {x2, y2});
    std::cout << "Path length (A*): " << (int)pathAstar.size() - 1 << "\n";
    colorMap.print(pathAstar);

    return 0;
}
#include "Map.h"
#include "Search.h"
#include "ColorMap.h"
#include <iostream>
#include <string>
#include <stdexcept>

using namespace std;

// Returns true if the string represents a valid integer
bool isInteger(const string& s){
    if(s.empty()) return false;
    size_t start = (s[0] == '-') ? 1 : 0;
    for(size_t i = start; i < s.size(); i++)
        if(!isdigit(s[i])) return false;
    return true;
}

int main(int argc, char *argv[]){


    if(argc != 7){
        cerr << "Uso: " << argv[0]
                  << " <mapa.txt> <fila_inicio> <col_inicio> <fila_fin> <col_fin> [altura]\n";
        return 1;
    }


    for(int i = 2; i <= 5; i++){
        if(!isInteger(argv[i])){
            cerr << "Error: el argumento " << i
                      << " ('" << argv[i] << "') no es un entero valido.\n";
            return 1;
        }
    }

    string mapFile = argv[1];
    int x1 = atoi(argv[2]);
    int y1 = atoi(argv[3]);
    int x2 = atoi(argv[4]);
    int y2 = atoi(argv[5]);


    Map map(mapFile);
    if(map == Map()){         
        cerr << "Error: no se pudo cargar el mapa '" << mapFile << "'.\n";
        return 1;
    }
    if(x1 < 0 || x1 >= map.get_h() || y1 < 0 || y1 >= map.get_w() ||
       x2 < 0 || x2 >= map.get_h() || y2 < 0 || y2 >= map.get_w()){
        std::cerr << "Error: coordenadas fuera del mapa" << std::endl;
        return 1;
    }

   bool esAltura = (argc==7 && string(argv[6]) == "altura");

   if(esAltura){
        cout << "Modo Altura Activado" << endl;
    }
    else{
        cout << "Modo Altura Desactivado" << endl;
    }
        
 
    ColorMap colorMap(map);

    // Print the raw map
    std::cout << "Mapa cargado:\n";
   // colorMap.print();


    auto pathBFS = Search::BFS(map, {x1, y1}, {x2, y2});
    cout << "Pasos:" << pathBFS.size() - 1 << endl;
    cout << "Costo total: " << Search::calcularLargoCamino(map, pathBFS, esAltura) << endl;
    // Path length = number of steps (edges), which is nodes - 1
    //std::cout << "Path length (BFS): " << (int)pathBFS.size() - 1 << "\n";
    //colorMap.print(pathBFS);


    auto pathGreedy = Search::Greedy(map, {x1, y1}, {x2, y2});
    cout << "Pasos:" << pathGreedy.size() - 1 << endl;
    cout << "Costo total: " << Search::calcularLargoCamino(map, pathGreedy, esAltura) << endl;
    //std::cout << "Path length (Greedy): " << (int)pathGreedy.size() - 1 << "\n";
    //colorMap.print(pathGreedy);


    auto pathAstar = Search::Astar(map, {x1, y1}, {x2, y2}, esAltura);
    cout << "Pasos:" << pathAstar.size() - 1 << endl;
    cout << "Costo total: " << Search::calcularLargoCamino(map, pathAstar, esAltura) << endl;
    //std::cout << "Path length (A*): " << (int)pathAstar.size() - 1 << "\n";
    //colorMap.print(pathAstar);

    auto pathWAstar = Search::WAstar(map, {x1, y1}, {x2, y2}, 1.5f, esAltura);
    cout << "Pasos:" << pathWAstar.size() - 1 << endl;
    cout << "Costo total: " << Search::calcularLargoCamino(map, pathWAstar, esAltura) << endl;
    //std::cout << "Path length (Weighted A*): " << (int)pathWAstar.size() - 1 << "\n";
    //colorMap.print(pathWAstar);

    return 0;
}
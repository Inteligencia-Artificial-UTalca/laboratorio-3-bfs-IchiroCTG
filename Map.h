#pragma once
#include <string>
#include <vector>

// clase que representa el mapa del laberinto
class Map{

protected:
    int h; // filas del mapa
    int w; // columnas del mapa
    std::vector<std::vector<int>> _map; // matriz con los valores del mapa

public:
    Map();                              // constructor vacio
    Map(std::string filename);          // carga el mapa desde un archivo
    Map(const Map& rhs);                // constructor copia
    ~Map();                             // destructor
    Map& operator=(const Map& rhs);     // operador de asignacion

    void print() const;                               // imprime el mapa
    void print(std::vector<std::pair<int,int>>) const; // imprime el mapa con el camino
    int get_h();
    int get_w(); // retorna el valor del mapa en la posicion (x,y)
    friend bool operator==(const Map& lhs, const Map& rhs); // para comparar dos mapas
    friend class Search; // le doy acceso a Search para que pueda leer h, w y _map
};

// retorna true si los dos mapas son iguales
bool operator==(const Map& lhs, const Map& rhs);
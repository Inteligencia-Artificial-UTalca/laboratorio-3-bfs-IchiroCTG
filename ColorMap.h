#pragma once
#include "Map.h"
#include <array>

// hereda de Map y reimplementa print para imprimir con colores en consola
class ColorMap : public Map{

    // colores ANSI para cada valor del mapa
    // 0=blanco, 1=azul, 2=verde, 3=rojo, 4=amarillo
    std::array<int,5> colors;

public:
    ColorMap(const Map& rhs); // recibe un mapa normal y lo convierte a color

    void print() const;                              // imprime el mapa con colores
    void print(std::vector<std::pair<int,int>>) const; // imprime el mapa y el camino con colores
};
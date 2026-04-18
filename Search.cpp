#include "Search.h"
#include <queue>
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <cmath>
#include <functional>

namespace std
{
    // Needed to store pair<int,int> in unordered containers
    template<> struct hash<std::pair<int,int>>
    {
        std::size_t operator()(const std::pair<int,int>& p) const noexcept
        {
            hash<int> hasher;
            return hasher(p.first) ^ (hasher(p.second) << 1);
        }
    };
}

// ─────────────────────────────────────────────
//  reconstruct
//  Travels the pathCache map from goal back to
//  start, then reverses the result.
// ─────────────────────────────────────────────
std::vector<std::pair<int,int>> Search::reconstruct(
    const std::unordered_map<std::pair<int,int>,std::pair<int,int>>& pathCache,
    const std::pair<int,int>& start)
{
    std::deque<std::pair<int,int>> nodes;
    auto node = start; // 'start' here is actually the GOAL node

    while(true){
        nodes.push_front(node);
        auto it = pathCache.find(node);
        if(it == pathCache.end()) break; // reached the start (no parent)
        node = it->second;               // move to parent
    }

    std::vector<std::pair<int,int>> vec(nodes.begin(), nodes.end());
    return vec;
}

// ─────────────────────────────────────────────
//  Heuristic  (Manhattan distance)
// ─────────────────────────────────────────────
float Search::Heuristic(std::pair<int,int> current, std::pair<int,int> goal)
{
    return std::abs(current.first - goal.first) +
           std::abs(current.second - goal.second);
}

// ─────────────────────────────────────────────
//  BFS  (Lab 3)
//  Guarantees shortest path in steps.
// ─────────────────────────────────────────────
std::vector<std::pair<int,int>> Search::BFS(
    const Map& map,
    std::pair<int,int> start,
    std::pair<int,int> goal)
{
    std::cout << "===========================\nRunning BFS...\n";
    auto startTime = std::chrono::high_resolution_clock::now();

    std::pair<int,int> dirs[]{{-1,0},{0,1},{1,0},{0,-1}};

    bool visited[map.h][map.w];
    for(int i = 0; i < map.h; i++)
        for(int j = 0; j < map.w; j++)
            visited[i][j] = false;

    std::queue<std::pair<int,int>> OPEN;
    std::unordered_map<std::pair<int,int>,std::pair<int,int>> pathCache; // child -> parent

    OPEN.push(start);
    visited[start.first][start.second] = true;

    while(!OPEN.empty()){
        auto pos = OPEN.front();
        OPEN.pop();

        if(pos == goal){
            auto endTime = std::chrono::high_resolution_clock::now();
            int count = 0;
            for(int i = 0; i < map.h; i++)
                for(int j = 0; j < map.w; j++)
                    if(visited[i][j]) count++;
            std::cout << "VISITED: " << count << std::endl;
            std::cout << "OPEN: "    << OPEN.size() << std::endl;
            std::cout << "FOUND in " << (endTime-startTime).count()/1000000.0 << "ms\n";
            return reconstruct(pathCache, pos);
        }

        for(auto dir : dirs){
            std::pair<int,int> next = {pos.first + dir.first, pos.second + dir.second};

            // Bounds check
            if(next.first < 0 || next.first >= map.h ||
               next.second < 0 || next.second >= map.w)
                continue;

            // Wall or already visited
            if(map._map[next.first][next.second] == 1) continue;
            if(visited[next.first][next.second])        continue;

            visited[next.first][next.second] = true;
            pathCache[next] = pos;
            OPEN.push(next);
        }
    }

    std::cout << "NOT FOUND!!!!\n";
    std::vector<std::pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}

// ─────────────────────────────────────────────
//  Greedy Best-First Search  (Lab 4)
//  Expands the node with the lowest h(n).
// ─────────────────────────────────────────────
std::vector<std::pair<int,int>> Search::Greedy(
    const Map& map,
    std::pair<int,int> start,
    std::pair<int,int> goal)
{
    std::cout << "===========================\nRunning Greedy BFS...\n";
    auto startTime = std::chrono::high_resolution_clock::now();

    std::pair<int,int> dirs[]{{-1,0},{0,1},{1,0},{0,-1}};

    // Priority queue: min-heap by heuristic value
    // Element: { h_value, position }
    using Node = std::pair<float, std::pair<int,int>>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> OPEN;

    std::unordered_map<std::pair<int,int>, std::pair<int,int>> pathCache; // child -> parent
    std::unordered_map<std::pair<int,int>, bool> visited;

    OPEN.push({Heuristic(start, goal), start});
    visited[start] = true;

    while(!OPEN.empty()){
        std::pair<float, std::pair<int,int>> top = OPEN.top();
        OPEN.pop();
        std::pair<int,int> pos = top.second;

        if(pos == goal){
            auto endTime = std::chrono::high_resolution_clock::now();
            std::cout << "VISITED: " << visited.size() << std::endl;
            std::cout << "OPEN: "    << OPEN.size() << std::endl;
            std::cout << "FOUND in " << (endTime-startTime).count()/1000000.0 << "ms\n";
            return reconstruct(pathCache, pos);
        }

        for(auto dir : dirs){
            std::pair<int,int> next = {pos.first + dir.first, pos.second + dir.second};

            if(next.first < 0 || next.first >= map.h ||
               next.second < 0 || next.second >= map.w)
                continue;

            if(map._map[next.first][next.second] == 1) continue;
            if(visited.count(next))                     continue;

            visited[next] = true;
            pathCache[next] = pos;
            OPEN.push({Heuristic(next, goal), next});
        }
    }

    std::cout << "NOT FOUND!!!!\n";
    std::vector<std::pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}

// ─────────────────────────────────────────────
//  A*  (Lab 5)
//  Expands the node with the lowest f = g + h.
// ─────────────────────────────────────────────
std::vector<std::pair<int,int>> Search::Astar(
    const Map& map,
    std::pair<int,int> start,
    std::pair<int,int> goal)
{
    std::cout << "===========================\nRunning A*...\n";
    auto startTime = std::chrono::high_resolution_clock::now();

    std::pair<int,int> dirs[]{{-1,0},{0,1},{1,0},{0,-1}};

    // Priority queue: min-heap by f = g + h
    using Node = std::pair<float, std::pair<int,int>>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> OPEN;

    std::unordered_map<std::pair<int,int>, std::pair<int,int>> pathCache; // child -> parent
    std::unordered_map<std::pair<int,int>, float> gCost;                  // real cost so far
    std::unordered_map<std::pair<int,int>, bool>  closed;                 // closed list

    gCost[start] = 0.0f;
    OPEN.push({Heuristic(start, goal), start});

    while(!OPEN.empty()){
        std::pair<float, std::pair<int,int>> top = OPEN.top();
        OPEN.pop();
        std::pair<int,int> pos = top.second;

        if(closed.count(pos)) continue; // already expanded with lower cost
        closed[pos] = true;

        if(pos == goal){
            auto endTime = std::chrono::high_resolution_clock::now();
            std::cout << "VISITED: " << closed.size() << std::endl;
            std::cout << "OPEN: "    << OPEN.size() << std::endl;
            std::cout << "FOUND in " << (endTime-startTime).count()/1000000.0 << "ms\n";
            return reconstruct(pathCache, pos);
        }

        for(auto dir : dirs){
            std::pair<int,int> next = {pos.first + dir.first, pos.second + dir.second};

            if(next.first < 0 || next.first >= map.h ||
               next.second < 0 || next.second >= map.w)
                continue;

            if(map._map[next.first][next.second] == 1) continue;
            if(closed.count(next))                      continue;

            float tentative_g = gCost[pos] + 1.0f; // each step costs 1

            // Only add to OPEN if we found a better (or first) path to 'next'
            if(!gCost.count(next) || tentative_g < gCost[next]){
                gCost[next]    = tentative_g;
                pathCache[next] = pos;
                float f = tentative_g + Heuristic(next, goal);
                OPEN.push({f, next});
            }
        }
    }

    std::cout << "NOT FOUND!!!!\n";
    std::vector<std::pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}
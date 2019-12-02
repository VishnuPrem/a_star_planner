#include <iostream>
#include<unordered_set>
#include <vector>
#include<cmath>

#define ROW 20
#define COL 20


struct Cell{

    Cell():x(-1),y(-1), g(0.0), f(0.0), occupancy(0), parent(nullptr){};
    int x;
    int y;
    float g;
    float f;
    int occupancy; // 0: unoccupied 1: obstacle 8:start 9:goal
    Cell* parent;

};


class Map{

private:


    int start_x;
    int start_y;
    int goal_x;
    int goal_y;

public:

    Cell map[ROW][COL];

    Map(int s_x,int s_y, int g_x, int g_y){

        for(int i = 0;i<ROW;i++)
            for(int j=0;j<COL; j++)
            {
                map[i][j].x = i;
                map[i][j].y = j;
                if(i==0 || j==0 || i==ROW-1 || j == COL-1)
                    map[i][j].occupancy = 1;

            }

        start_x = s_x;
        start_y = s_y;
        goal_x = g_x;
        goal_y = g_y;

        map[start_x][start_y].occupancy = 8;
        map[goal_x][goal_y].occupancy = 9;


    }

    void add_obstacles(int x1, int y1, int x2, int y2)
    {
        if(x1>x2 || y1>y2)
            return;
        for(int i = x1; i<x2; i++)
            for(int j = y1; j<y2; j++)
                map[i][j].occupancy = 1;
    }

    void display_map() {

        std::cout << "\n\n";
        for(int i = 0; i < ROW; i++)
            std::cout << i%10 <<" ";
        std::cout<<std::endl;
        for (int i = 0; i < ROW; i++)
        {

            for (int j = 0; j < COL; j++) {
                switch (map[i][j].occupancy) {
                    case 0:
                        std::cout << ".";     //unoccupied
                        break;
                    case 1:
                        std::cout << "X";     //obstacle
                        break;
                    case 7:
                        std::cout << "P";      //path
                        break;
                    case 8:
                        std::cout << "S";     //start
                        break;
                    case 9:
                        std::cout << "G";     //goal
                        break;

                }
                std::cout << " ";
                if (j == COL - 1)
                    std::cout << " :" << i<<std::endl;
            }

        }
    }
    int get_start_x()
    {
        return start_x;
    }

    int get_start_y()
    {
        return start_y;
    }

    int get_goal_x()
    {
        return goal_x;
    }

    int get_goal_y()
    {
        return goal_y;
    }

};

class A_star_planner{



public:

    void get_plan(Map& m)
    {
        std::unordered_set<Cell*> open_list{};
        std::unordered_set<Cell*> closed_list{};

        Cell* start_ptr = &m.map[m.get_start_x()][m.get_start_y()];
        Cell* goal_ptr = &m.map[m.get_goal_x()][m.get_goal_y()];

        //add start to open
        open_list.insert(start_ptr);
        start_ptr->f = get_distance(start_ptr,goal_ptr);

        while(!open_list.empty())
        {
            Cell* cell = get_cell_with_min_f(m,open_list);

            if(cell == goal_ptr)
            {
                // success
                get_path(goal_ptr, m);

            }

            open_list.erase(cell);
            closed_list.insert(cell);

            std::vector<Cell*> neighbours = get_neighbours(m,cell);

            for(auto& neighbour: neighbours)
            {
                if(closed_list.find(neighbour)!=closed_list.end())
                    continue;

                float tentative_cost = cell->g + get_distance(cell,neighbour);

                if(open_list.find(neighbour)!= open_list.end())
                {
                    if(neighbour->g > tentative_cost)
                    {
                        neighbour->parent = cell;
                        neighbour->g = tentative_cost;
                        neighbour->f = neighbour->g + get_distance(neighbour,goal_ptr);
                    }
                }
                else
                {
                    neighbour->parent = cell;
                    neighbour->g = tentative_cost;
                    neighbour->f = neighbour->g + get_distance(neighbour,goal_ptr);
                    open_list.insert(neighbour);
                }
            }
        }
        // failure
        std::cout<<"failure";

    }

    Cell* get_cell_with_min_f(Map& m, std::unordered_set<Cell*>& open_list)
    {
        Cell* min_cell = nullptr;

        for(int i=0; i<ROW ; i++)
            for(int j=0; j<COL; j++)
                if(open_list.find(&m.map[i][j]) != open_list.end())
                {
                    if (min_cell == nullptr)
                    {
                        min_cell = &m.map[i][j];
                    }
                    else if (m.map[i][j].f < min_cell->f)
                    {
                        min_cell = &m.map[i][j];
                    }
                }
        if (min_cell == nullptr)
            std::cout<<"\n Min not found";

        return min_cell;
    }


    std::vector<Cell*> get_neighbours(Map& m, Cell* c)
    {
        std::vector<Cell*> neighbours;

        int neighbour_indices[8][2] = {{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1}};

        for(int i=0; i<8; i++)
        {
            int x = c->x + neighbour_indices[i][0];
            int y = c->y + neighbour_indices[i][1];

            if (m.map[x][y].occupancy != 1)
            {
                neighbours.push_back(&m.map[x][y]);
            }

        }

        return neighbours;
    }

    float get_distance(Cell* a, Cell* b)
    {
        return sqrt(pow(a->x-b->x,2) + pow(a->y-b->y,2));
    }

    void get_path(Cell* cell, Map &m)
    {

        while(cell->parent!= nullptr)
        {
            //path.push_back({cell->x,cell->y});
            std::cout<<cell->x<<" "<<cell->y<<std::endl;
            cell = cell->parent;
            m.map[cell->x][cell->y].occupancy = 7;
        }

    }


};

int main() {

    int start_x = 7;
    int start_y = 5;
    int goal_x = 14;
    int goal_y = 13;

    Map map1(start_x,start_y,goal_x,goal_y);

    map1.add_obstacles(6,6,10,10);
    map1.add_obstacles(12,1,14,15);
    map1.add_obstacles(4,15,16,17);

    map1.display_map();

    A_star_planner planner;
    planner.get_plan(map1);

    map1.display_map();

}
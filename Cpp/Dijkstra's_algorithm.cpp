#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

struct element{
  int g = 0;
  int p_r = 0;
  int p_c = 0;
  int sta = 0;                  //0 for not visited, 1 for open list, 2 for closed list
};

int find_min_f(vector <int> r, vector <int> c, element** n )
    {
    int min = 10000;
    int no = 0;
    for(int i = 0; i < r.size() ; i++)
        {
        if (n[r[i]][c[i]].g < min)
            {
            min = n[r[i]][c[i]].g;
            no = i;   
        }
    }
    return no;
}

void Dijksrta( int r, int c, int pacman_r, int pacman_c, int food_r, int food_c, vector <string> grid){
    element** node = new element*[r];
    for(int i = 0; i < r; i++)
        {
        node[i] = new element[c];
    }
    
    node[pacman_r][pacman_c].p_r = -1;  // make the parent of staring position to be -1 (this will be hwlpful while backtracking)
    node[pacman_r][pacman_c].p_c = -1;
    
    vector <int> path_r;
    vector <int> path_c;
    
    vector <int> open_r;            // list of open members
    vector <int> open_c;
    
    int move_r[] = {1,-1,0,0};             // the movement array
    int move_c[] = {0,0,1,-1};
    
    open_r.push_back(pacman_r);             // open list
    open_c.push_back(pacman_c);
    
    while(node[food_r][food_c].sta != 2) // search until the goal node is added to closed list
        {
        int min_f = find_min_f(open_r, open_c, node);       // find index of node with minimum f value
        int min_r = open_r[min_f];      // row value corresponding to min f
        int min_c = open_c[min_f];      // coloumn value corresponding to min f
        
        open_r.erase(open_r.begin() + min_f);
        open_c.erase(open_c.begin() + min_f);
        node[min_r][min_c].sta = 2;          // add the new node to closed list
        
        for(int i = 0; i < 4 ; i++ )     // search for each neighbour
            {
            int R = min_r + move_r[i];
            int C = min_c + move_c[i];
            
            if(R < r && R >=0 && C < c && C >=0 && (grid[R][C] == 45 || grid[R][C] == 46) && node[R][C].sta != 2)
                {
                if(node[R][C].sta == 0)
                    {
                    node[R][C].p_r = min_r;
                    node[R][C].p_c = min_c;
                    node[R][C].g = node[min_r][min_c].g + 1;
                    
                    node[R][C].sta = 1;
                    open_r.push_back(R);
                    open_c.push_back(C);
                    //cout <<"for R= "<< min_r<<" C="<<min_c<<" " <<R << " " << C << endl;
                }
                else
                    {
                    if(node[R][C].g > node[min_r][min_c].g + 1 )
                        {
                        node[R][C].p_r = min_r;
                        node[R][C].p_c = min_c;
                        node[R][C].g = node[min_r][min_c].g + 1;
                        //cout <<"for R= "<< min_r<<" C="<<min_c<<" " <<R << " " << C << endl;
                    }
                }
            }
        }
    }
    int R = food_r;
    int C = food_c;
    int dist;
    while(R != -1)
        {
       
        path_r.push_back(R);
        path_c.push_back(C);
        int pre_R = R;
        int pre_C = C;
        R = node[pre_R][pre_C].p_r;
        C = node[pre_R][pre_C].p_c;
        
    }
    dist = path_r.size() - 1;                //if two elements are there in path the path length is 1
    cout<< dist << endl;
    for(int i = 0; i <= dist  ; i++)
    {
    	cout << path_r[dist-i] << " " << path_c[dist-i]<< endl;
    }
    
}
int main(void) {

    int r,c, pacman_r, pacman_c, food_r, food_c;
    
    cin >> pacman_r >> pacman_c;
    cin >> food_r >> food_c;
    cin >> r >> c;
    
    vector <string> grid;

    for(int i=0; i<r; i++) {
        string s; cin >> s;
        grid.push_back(s);
    }

    Dijksrta( r, c, pacman_r, pacman_c, food_r, food_c, grid);

    return 0;
}

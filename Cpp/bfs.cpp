#include <iostream>
#include <vector>
#include <queue>
#include <stack>
using namespace std;
struct element
    {
    int p_c;
    int p_r;
    int sta = 0;                  //0 unsearched , 1 in que, 2 explored
};
void nextMove( int r, int c, int pacman_r, int pacman_c, int food_r, int food_c, vector <string> grid){
    //your logic here
    element** node = new element*[r];       // creating 2 D grid of nodes
    for(int i = 0; i < c; i++)
        {
        node[i] = new element[c];
    }
    queue <int> open_r;                     // to store the node in stack
    queue <int> open_c;
    vector <int> path_r;
    vector <int> path_c;
    
    int move_r[] = {-1, 0, 0 , 1};          // defininig moves (here 4 moves up, down, left, right)
    int move_c[] = {0, -1, 1, 0};
    
    int exp = 0;                           // to store the no of nodes expanded
    node[pacman_r][pacman_c].p_r = -1;
    node[pacman_r][pacman_c].p_c = -1;
    open_r.push(pacman_r);
    open_c.push(pacman_c);
    
    while(node[food_r][food_c].sta !=2  && !open_r.empty()) // continue until the food node is expanded or the queue is empty
        {
        int R = open_r.front();
        int C = open_c.front();
        //cout << R << " "<< C << endl;
        open_r.pop();
        open_c.pop();
        node[R][C].sta = 2;
        
        exp++;
        for(int i = 0 ; i < 4 ; i++)
            {
            int R_r = R + move_r[i];
            int C_c = C + move_c[i];
            if(R_r < r && C_c < c && R_r >=0 && C_c >= 0 && (grid[R_r][C_c] == 45 || grid[R_r][C_c] == 46) && node[R_r][C_c].sta == 0)
                {
                node[R_r][C_c].sta = 1;
                open_r.push(R_r);
                open_c.push(C_c);
                node[R_r][C_c].p_r = R;
                node[R_r][C_c].p_c = C;
            }
        }
    }
    
    int R = food_r;
    int C = food_c;
    while(R!=-1)            // tracing the path backwards
        {
        path_r.push_back(R);
        path_c.push_back(C);
        int pre_R = R;
        int pre_C = C;
        //cout << R << " " << C << endl ;
        R = node[pre_R][pre_C].p_r;
        C = node[pre_R][pre_C].p_c;
    }
    
    int path_len = path_r.size() - 1;  // printing the path length and steps
    cout << path_len<< endl;
    for(int i = 0 ; i < path_len + 1 ; i++)
        {
        cout <<path_r[path_len-i] << " " << path_c[path_len-i] <<endl;
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

    nextMove( r, c, pacman_r, pacman_c, food_r, food_c, grid);

    return 0;
}

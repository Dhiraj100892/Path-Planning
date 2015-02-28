#include <iostream>
#include <vector>
#include <stack>
using namespace std;
void dfs( int r, int c, int pacman_r, int pacman_c, int food_r, int food_c, vector <string> grid){
    //your logic here
    vector <int> v_r;
    vector <int> v_c;
    stack <int> V_R;
    stack <int> V_C;
    int i , R, C, visit, food_f;
    visit = 0;
    food_f = 0;
    int l[] = {0,0,-1,1};
    int k[] = {1,-1,0,0};
    
    int** graph = new int*[r];      // to store wheather node is visited or not (if visited then takes value -1) 
    for(i = 0; i < r; i++)
        {
        graph[i] = new int[c];
    }
    v_c.push_back(pacman_c);        // not necessary just use to print the path in order
    v_r.push_back(pacman_r);
    V_C.push(pacman_c);
    V_R.push(pacman_r);
    while(!V_C.empty())               // try until the stack is empty
        {
        visit = 0;
        R = V_R.top();
        C = V_C.top();
        graph[R][C] = -1;
        for(i = 0; i  < 4; i++)         // search for each neighbour
            {
            if( R+l[i] >= 0 && R+l[i] < r && C+k[i] >=0 && C+k[i] < c && graph[R+l[i]][C+k[i]]!=-1 && (grid[R+l[i]][C+k[i]] == 45 || grid[R+l[i]][C+k[i]] == 46))
            {
            visit = 1;    
            V_R.push(R+l[i]);
            V_C.push(C+k[i]);
            v_r.push_back(R+l[i]);
            v_c.push_back(C+k[i]);
            //cout << R+l[i] << " " << C+k[i] << endl;
            if(R+l[i] == food_r && C+k[i] == food_c)  food_f = 1;
            break;     
        }
        }
        if(food_f) 
        {
           // cout << V_R.size()<<end; 
            break;
            
        }
        if(visit = 0)                   // no neighbour pop the element from the stack
            {
            V_R.pop();
            V_C.pop();
            v_r.pop_back();
            v_c.pop_back();
        }
    }
    //cout << endl;
    int s = v_r.size();
    cout << s - 1 << endl;
    for(i = 0; i < s ; i++)
        {
        cout << v_r[i] <<" " << v_c[i] << endl;
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

    dfs( r, c, pacman_r, pacman_c, food_r, food_c, grid);

    return 0;
}

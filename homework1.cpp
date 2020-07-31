#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#include <set>
#include <deque>
#include <math.h>
#include <cmath>
#include <algorithm>

struct Node{
    int x, y;
    std::vector<Node> parent;
    bool const operator==(const Node& ob) const{
        return x == ob.x && y == ob.y;
    }
    bool operator<(const Node& ob) const{
        return x < ob.x || (x == ob.x && y < ob.y);
    }
};

struct PNode{
    int a,b,p;
    std::vector<PNode> parent;
    bool const operator==(const PNode& ob) const{
        return a == ob.a && b == ob.b;
    }
    bool operator<(const PNode& ob) const{
        return a < ob.a || (a == ob.a && b < ob.b);
    }
};
struct comparePriority{
    bool operator()(PNode const& n1, PNode const& n2){
        return n1.p>n2.p;
    }
};


struct ANode{
    int ax,ay;
    int gcost,fcost,hcost;
    std::vector<ANode>parent;
    bool const operator==(const ANode& ob) const{
        return ax == ob.ax && ay == ob.ay;
    }
    bool operator<(const ANode& ob) const{
        return ax < ob.ax || (ax == ob.ax && ay < ob.ay);
    }
};
struct compareAPriority{
    bool operator()(ANode const& n1, ANode const& n2){
        return n1.fcost>n2.fcost;
    }
};

int W,H,Z;
std::queue<Node> q;
std::ofstream outputfile;
std::priority_queue <PNode, std::vector<PNode>, comparePriority> pq;
std::vector<ANode> open;
std::set<ANode> closed;

int dy[] = {-1, -1, -1, 0, 1, 0, 1, 1};
int dx[] = {-1, 1, 0, -1, -1, 1, 0, 1};
std::set<Node> visited;
std::set<PNode> uvisited;

std::vector<std::vector<int>> target;
std::vector<std::vector<int>> map;

void astarcost(ANode curr,int b) {
    int yy,xx;
    for (int i = 0; i < 8; i++){
        yy=curr.ay+dy[i];
        xx=curr.ax+dx[i];
        if (xx<0||yy<0)
        continue;
        if (xx>=W||yy>=H)
        continue;
        if (abs(map[yy][xx]-map[curr.ay][curr.ax])>Z)
        continue;
        
        ANode next = {xx,yy,curr.gcost,0,0,{curr}};
        
        if (closed.count(next))
        continue;
        
        if(abs(dy[i])&&abs(dx[i]))
            next.gcost+=abs(map[yy][xx]-map[curr.ay][curr.ax])+14;
        else
            next.gcost+=abs(map[yy][xx]-map[curr.ay][curr.ax])+10;
        
        next.hcost=std::max (abs(next.ax-target[b][0]),abs(next.ay-target[b][1]));
        next.fcost=next.gcost+next.hcost;

        auto it = std::find(open.begin(),open.end(),next);
        if(it!=open.end()){
            if(it->gcost>next.gcost){
                it->gcost=next.gcost;
                it->fcost=next.fcost;
                it->parent=next.parent;
                }
                std::make_heap(open.begin(),open.end(),compareAPriority());
                }
                else{
                    open.push_back(next);
                    std::push_heap(open.begin(),open.end(),compareAPriority());
                    }
            }
        }

void aprintPath(std::vector<ANode> path){
    if (path.size() == 0)
    return;
    aprintPath(path[0].parent);
    std::cout <<path[0].ax << "," <<path[0].ay << " " ;
    outputfile <<path[0].ax << "," <<path[0].ay << " " ;
}

void Astar(int X,int Y,int i) {
    while(!open.empty())
    open.pop_back();
    closed.clear();
    bool reached_end=false;
    int hrsc=std::max (abs(X-target[i][0]),abs(Y-target[i][1]));
    int frsc=hrsc;
    ANode src={X,Y,0,frsc,hrsc};
    ANode curr;
    open.push_back(src);
   
   std::make_heap(open.begin(),open.end(),compareAPriority());
    while(!open.empty()){
        curr=open.front();
        pop_heap(open.begin(),open.end());
        open.pop_back();
        closed.insert(curr);
        if ((curr.ax==target[i][0])&&(curr.ay==target[i][1])){
            reached_end=true;
            break;
        }
        astarcost(curr,i);
    }
    if (reached_end){
        aprintPath({curr});
        std::cout << '\n';
    }
    else{
        std::cout << "FAIL" << '\n';
        outputfile <<"FAIL";
    }
}

void explore_neighbours(Node curr){
    int yy,xx;
    for (int i = 0; i < 8; i++) {
        yy=curr.y+dy[i];
        xx=curr.x+dx[i];
        if (xx<0||yy<0)
        continue;
        if (xx>=W||yy>=H)
        continue;
        if (abs(map[yy][xx]-map[curr.y][curr.x])>Z)
        continue;
        Node next = {xx,yy,{curr}};
        if (visited.count(next))
        continue;
        q.push(next);
        visited.insert(next);
        //nodes_in_next_layer++;
    }
}
void ucs_cost(PNode curr){
    int yy,xx;
    for (int i = 0; i < 8; i++) {
        yy=curr.b+dy[i];
        xx=curr.a+dx[i];
        if (xx<0||yy<0)
        continue;
        if (xx>=W||yy>=H)
        continue;
        if (abs(map[yy][xx]-map[curr.b][curr.a])>Z)
        continue;
        PNode next = {xx,yy,curr.p,{curr}};
        if (uvisited.count(next))
        continue;
        if(abs(dy[i])&&abs(dx[i])){
            next.p+=14;
            pq.push(next);
        }
        else{
            next.p+=10;
            pq.push(next);
        }
        uvisited.insert(next);
    }
}

void printPath(std::vector<Node> path){
    if (path.size() == 0)
    return;
    printPath(path[0].parent);
    std::cout <<path[0].x << "," <<path[0].y << " " ;
    outputfile <<path[0].x << "," <<path[0].y << " " ;
    
}

void uprintPath(std::vector<PNode> path){
    if (path.size() == 0)
    return;
    uprintPath(path[0].parent);
    std::cout <<path[0].a << "," <<path[0].b << " " ;
    // std::cout<<pathcost;
    outputfile <<path[0].a << "," <<path[0].b << " " ;
}

void UCS(int X,int Y,int i){
    while(!pq.empty()) pq.pop();
    uvisited.clear();
    bool reached_end=false;
    PNode src = {X,Y,0};
    PNode curr;
    pq.push(src);
    uvisited.insert(src);
    while (!pq.empty()) {
        curr=pq.top();
        pq.pop();
        if ((curr.a==target[i][0])&&(curr.b==target[i][1])){
            reached_end=true;
            break;
        }
        ucs_cost(curr);
    }
    if (reached_end){
        uprintPath({curr});
        std::cout << '\n';
    }
    else{
        std::cout << "FAIL" << '\n';
        outputfile <<"FAIL";
    }
}

void BFS(int X,int Y,int i) {
    //int nodes_left_in_layer=1,move_count=0;
    while(!q.empty()) q.pop();
    visited.clear();
    bool reached_end=false;
    Node src = {X,Y};
    Node curr;
    q.push(src);
    visited.insert(src);
    while (!q.empty()) {
        curr=q.front();
        q.pop();
        if ((curr.x==target[i][0])&&(curr.y==target[i][1])){
            reached_end=true;
            break;
        }
        explore_neighbours(curr);
    }
    if (reached_end){
        printPath({curr});
        std::cout << '\n';
    }
    else{
        std::cout << "FAIL" << '\n';
        outputfile <<"FAIL";
    }
}

int main() {
    std::ifstream inFile;
    inFile.open("testcases/input4.txt");
    if (!inFile) {
        std::cerr << "Unable to open file" << '\n';
        exit(1);
    }
    outputfile.open("output.txt");
    std::string algo;
    int X,Y,N,arr[2],map1;
    inFile>>algo;
    inFile>>W>>H;
    inFile>>X>>Y;
    inFile>>Z;
    inFile>>N;
    for ( int i = 0; i < N; i++) {
        inFile>>arr[0]>>arr[1];
        std::vector<int> temp(arr,arr+2);
        target.push_back(temp);
    }
    for ( int i = 0; i < H; i++) {
        std::vector<int> temp;
        for ( int j = 0; j < W; j++)
        {
            inFile>>map1;
            temp.push_back(map1);
        }
        map.push_back(temp);
    }
    if (algo=="BFS") {
        for (int i = 0; i < N; i++) {
            BFS(X,Y,i);
            outputfile<<"\n";
        }
    }
    if (algo=="UCS") {
        for (int i = 0; i < N; i++) {
            UCS(X,Y,i);
            outputfile<<"\n";
        }
    }
    if (algo=="A*") {
        for (int i = 0; i < N; i++) {
            Astar(X,Y,i);
            outputfile<<"\n";
        }
    }
    return 0;
}


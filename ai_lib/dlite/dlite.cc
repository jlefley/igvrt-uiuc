/**
D-lite version 1.

Kevin Chen
Leonardo Bobadilla

 */

#include <iostream>
#include <list>
#include <queue>
#include <math.h>

#define INF 1000000
using namespace std;



struct node {
  int info;
  node *next;
};





class State{
public:
  State();
  State(int occup);
  ~State();
  int getOccup();
  void print();
  void setRhs(const double rhs_value){rhs = rhs_value; }
  double getRhs () const{return rhs; }
  void setG(const double g_value){g = g_value; }
  double getG () const{return g; }
  double CalcKey();
  double h(State start); //The heuristic



private:
  double rhs;
  int x;
  int y;
  double g;
  int occupancy;
  bool goal;
  bool start;
  
  //  list<State> succesors; //a list with the succesors maybe some pointers
  State* succesors; //a list with the succesors maybe some pointers
  
  //maybe put the neighboors
};


State::State() {

   rhs=INF;
   g=INF;
   occupancy=0;
   goal=false;
   start=false;

}

State::State(int occup) {

   rhs=INF;
   g=INF;
   occupancy=occup;
   goal=false;
   start=false;

   }

int State:: getOccup()
{
  return occupancy;
}
void State:: print() {

  cout<< rhs <<endl; 
  cout<< g <<endl;

}

double State:: CalcKey(){

  return min(rhs,g);

}

double State:: h(State sgoal){

  return sqrt((pow ((x - sgoal.x) , 2)) + (pow ((y - sgoal.y) , 2))) ;

}





State::~State() {

}





class Map{
public:
  Map();
  Map(int matrix[4][4], int width, int length);
  ~Map();
  void printMap();
  void Initialize();
  void UpdateVertex(State s);
  void ComputeShortestPath();
private:
  list<State> states;
  list<State>::iterator it;
  queue<State> U;
  int width;
  int length;
};
   

Map:: Map(){
  
 

}

void Map ::  printMap(){
  it=states.begin();
 for (int i = 0; i< width; i++)
   {
   for (int j = 0; j< length; j++){
    it++;
    cout<<(*it).getOccup();
   }
   cout<<endl;
   }
 

}

Map:: Map(int  matrix[4][4], int width, int length){
 

 
it = states.begin();
 for (int i = 0; i< width; i++)
   for (int j = 0; j< length; j++){
     State s(matrix[i][j]);
      states.insert(it,s);
   }
      
}

Map:: ~Map(){
  
 

}



int main() {
 int minimap[4][4] = {{0,0,0,0},{0,1,0,0},{0,0,0,0},{0,0,1,0}};

  State s1;
  s1.print();

  Map m(minimap,4,4);
  

 
}using namespace std;


#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <string>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iterator>
#include <numeric>
using namespace std;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit functio
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

/*solid AssimpScene
 facet normal 0 0 -1
  outer loop
  vertex  0  0 0
  vertex 0   1 0
  vertex  1 0 0
  endloop
 endfacet
*/

float x[2000][3];
int up;

float w=3;    // center(hub) width, affects vertical blade width 
float b_w=3;    // blade width
float b_l=40;    // blade length
float b_t=b_w/4;    // blade thickness
float t_c=120*(2.0/3);    // height from vertical blade base to the ground

int main()
{

ofstream myfile;
myfile.open ("turbine.stl");

myfile << "solid AssimpScene.\n";

int cc;
cc=0;


x[cc][0]=w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c-b_w;

float m_s=1.5;

for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]+m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]+m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



///side elements

int k=b_l/m_s;


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]-m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]-m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]-b_w*0.8+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]-m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";

///

cc=0;
x[cc][0]=w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c-b_w;

for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]+m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]+m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



//side element 2




















////////////////////




cc=0;
x[cc][0]=-w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c-b_w;

for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]-m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]-m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}

//side element 2

cc=0;
k=b_l/m_s;


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]+m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]+m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]+m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]-b_w*0.8+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";




cc=500;
x[cc][0]=-w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c-b_w;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]-m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]-m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



//inclined surfaces//////////////



cc=0;
x[cc][0]=-w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]-m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]-m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}








///inclined 2




cc=0;
x[cc][0]=w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]+m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]+m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}




























////////////vertical 


cc=300;
x[cc][0]=-w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c;




//int theta=M_PI/5;
//int beta=M_PI/2 - theta;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}




cc=0;
x[cc][0]=-w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=+w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}




//// vertical 2



cc=300;
x[cc][0]=-w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c;




//int theta=M_PI/5;
//int beta=M_PI/2 - theta;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";

myfile << "endloop\n";
myfile << "endfacet\n";
}




cc=0;
x[cc][0]=w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}

myfile<< "endsolid AssimpScene";

myfile.close();

return 0;
}

// Passing all the variables inside the vector from the beginning of the vector to the end.
//scopy( lv.begin( ), lv.end( ), output_iterator );
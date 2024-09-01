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

float w=3;    // center(hub) width, affects vertical blade width 
float b_w=3;    // blade width
float b_l=3;    // blade length
float b_t=0;    // blade thickness
float t_c=80 + b_w;    // height from vertical blade base to the ground

float x[2000][3];
float m_s = 1.5;
int k = b_l/m_s;

int main()
{
    ofstream myfile;
    myfile.open ("turbine.stl");
    myfile << "solid AssimpScene.\n";

    // Pala izquierda
    int cc = 0;
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
        myfile << "facet normal 0 1 0\n";
        myfile << "outer loop\n";
        myfile << "vertex " << x[cc+2*k][0] << " " << x[cc+2*k][1] << " " << x[cc+2*k][2] << "\n";
        myfile << "vertex " << x[cc+2*k+2][0] << " " << x[cc+2*k+2][1] << " " << x[cc+2*k+2][2] << "\n";
        myfile << "vertex " << x[cc+2*k+1][0] << " " << x[cc+2*k+1][1] << " " << x[cc+2*k+1][2] << "\n";
        myfile << "endloop\n";
        myfile << "endfacet\n";
    }

    for (int k = 0; k < b_l/m_s; k++){
        myfile << "facet normal 0 1 0\n";
        myfile << "outer loop\n";
        myfile << "vertex " << x[cc+2*k+1][0] << " " << x[cc+2*k+1][1] << " " << x[cc+2*k+1][2] << "\n";
        myfile << "vertex " << x[cc+2*k+2][0] << " " << x[cc+2*k+2][1] << " " << x[cc+2*k+2][2] << "\n";
        myfile << "vertex " << x[cc+2*k+3][0] << " " << x[cc+2*k+3][1] << " " << x[cc+2*k+3][2] << "\n";
        myfile << "endloop\n";
        myfile << "endfacet\n";
    }

    // Pala derecha
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
        myfile << "facet normal 0 1 0\n";
        myfile << "outer loop\n";
        myfile << "vertex " << x[cc+2*k][0] << " " << x[cc+2*k][1] << " " << x[cc+2*k][2] << "\n";
        myfile << "vertex " << x[cc+2*k+1][0] << " " << x[cc+2*k+1][1] << " " << x[cc+2*k+1][2] << "\n";
        myfile << "vertex " << x[cc+2*k+2][0] << " " << x[cc+2*k+2][1] << " " << x[cc+2*k+2][2] << "\n";
        myfile << "endloop\n";
        myfile << "endfacet\n";
    }

    for (int k = 0; k < b_l/m_s; k++){
        myfile << "facet normal 0 1 0\n";
        myfile << "outer loop\n";
        myfile << "vertex " << x[cc+2*k+1][0] << " " << x[cc+2*k+1][1] << " " << x[cc+2*k+1][2] << "\n";
        myfile << "vertex " << x[cc+2*k+3][0] << " " << x[cc+2*k+3][1] << " " << x[cc+2*k+3][2] << "\n";
        myfile << "vertex " << x[cc+2*k+2][0] << " " << x[cc+2*k+2][1] << " " << x[cc+2*k+2][2] << "\n";
        myfile << "endloop\n";
        myfile << "endfacet\n";
    }

    // Pala vertical
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
        myfile << "facet normal 0 1 0\n";
        myfile << "outer loop\n";
        myfile << "vertex " << x[cc+2*k][0] << " " << x[cc+2*k][1] << " " << x[cc+2*k][2] << "\n";
        myfile << "vertex " << x[cc+2*k+2][0] << " " << x[cc+2*k+2][1] << " " << x[cc+2*k+2][2] << "\n";
        myfile << "vertex " << x[cc+2*k+1][0] << " " << x[cc+2*k+1][1] << " " << x[cc+2*k+1][2] << "\n";
        myfile << "endloop\n";
        myfile << "endfacet\n";
    }

    for (int k = 0; k < b_l/m_s; k++){
        myfile << "facet normal 0 1 0\n";
        myfile << "outer loop\n";
        myfile << "vertex " << x[cc+2*k+1][0] << " " << x[cc+2*k+1][1] << " " << x[cc+2*k+1][2] << "\n";
        myfile << "vertex " << x[cc+2*k+2][0] << " " << x[cc+2*k+2][1] << " " << x[cc+2*k+2][2] << "\n";
        myfile << "vertex " << x[cc+2*k+3][0] << " " << x[cc+2*k+3][1] << " " << x[cc+2*k+3][2] << "\n";
        myfile << "endloop\n";
        myfile << "endfacet\n";
    }

    myfile << "endsolid AssimpScene";

    myfile.close();

    return 0;
}
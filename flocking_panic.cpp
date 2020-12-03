#include <iostream>
#include <iomanip>
#include <vector>
#include <time.h>
#include <stdlib.h>
#include <cmath>
#include <algorithm> 
#include <functional>
#include <fstream>
#include <math.h>
#include <random>

using namespace std;

void force(vector<double>,vector<double>,vector<double>,vector<double>);
void buffer(double,vector<double>,vector<double>,vector<double>,vector<double>);

int N = 500;  // numero de agentes
int nb = 0;
vector <double> ramp(N,0.1);  //amplitud fuerza aleatoria
int n_iter = 1000;  // numero de iteraciones temporales
double dt = 0.01;  // paso temporal
double r0 = 0.025;  // rango de la fuerza de repulsion
double eps = 25;  // amplitud fuerza de repulsion
double rf = 0.1;  // rango de la fuerza de flocking
vector <double> alpha(N,1.0);  // amplitud de la fuerza de flocking
vector <double> v0(N,0.02);  // "target speed"
double mu = 10.0;  // amplitud fuerza de auto-propulsion
float panico = 0.10;

vector <double> xb(4*N,0);
vector <double> yb(4*N,0);
vector <double> vxb(4*N,0);
vector <double> vyb(4*N,0);
vector <double> fx(N,0);
vector <double> fy(N,0);

int main(){

    ofstream myDatax;
    myDatax.open("datos_x.txt");
  if (myDatax.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

        ofstream myDatay;
    myDatay.open("datos_y.txt");
  if (myDatay.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

        ofstream myDatavx;
    myDatavx.open("datos_vx.txt");
  if (myDatavx.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

        ofstream myDatavy;
    myDatavy.open("datos_vy.txt");
  if (myDatavy.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

    myDatax << fixed;
    myDatay << fixed;
    myDatavx << fixed;
    myDatavy << fixed;

    for (int i=0; i<ceil(N*panico); i++)
    {
        ramp[i] = 10.0;
        alpha[i] = 0.1; //Parametros distintos de los agentes en panico
        v0[i] = 0.05;
    }

    vector <double> x(N,0);
    vector <double> y(N,0);
    vector <double> vx(N,0);
    vector <double> vy(N,0);

    default_random_engine generator;
    uniform_real_distribution<double> distribution1(0.0,1.0);
    uniform_real_distribution<double> distribution2(-1.0,1.0);

    // inicializar posiciones y velocidades
    for(int j=0;j<N;j++){
        x[j] = distribution1(generator);  //posiciones aleatoria de 0 a 1
        y[j] = distribution1(generator);
        vx[j] = distribution2(generator);  // velocidad aleatoria de -1 a 1
        vy[j] = distribution2(generator);
    }
    // loop temporal
    for(int iterate=0;iterate<n_iter;iterate++){
        buffer(rf,x,y,vx,vy);
        force(x,y,vx,vy);

        for(int i=0;i<N;i++){
            vx[i] += fx[i]*dt;
            vy[i] += fy[i]*dt;
            x[i] += vx[i]*dt;
            y[i] += vy[i]*dt;
            x[i] = fmod((1.0+x[i]),1.0);
            y[i] = fmod((1.0+y[i]),1.0);
        }
        
        for (int h=0;h<N;h++){
            myDatax << setw(10) << x[h] << " ";
            myDatay << setw(10) << y[h] << " ";
            myDatavx << setw(10) << vx[h] << " ";
            myDatavy << setw(10) << vy[h] << " ";
        }
        myDatax << "\n";
        myDatay << "\n";
        myDatavx << "\n";
        myDatavy << "\n";
    }
    myDatax.close();
    myDatay.close();
    myDatavx.close();
    myDatavy.close();

    return 0;
}

void buffer(double rb,vector<double> x,vector<double> y,vector<double> vx,vector<double> vy)
{
    for (int i=0; i<N; i++)
    {
        xb[i] = x[i];
        yb[i] = y[i];
        vxb[i] = vx[i];
        vyb[i] = vy[i];
    }
    nb=N-1;

    for(int k=0;k<N;k++){
        if(x[k] <= rb){  // cerca a la izquierda
            nb+=1;
            xb[nb]= x[k] + 1;
            yb[nb] = y[k];
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(x[k]>=(1.0-rb)){  // cerca a la derecha
            nb+=1;
            xb[nb] = x[k] - 1;
            yb[nb] = y[k];
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(y[k] <= rb){  // cerca abajo
            nb+=1;
            yb[nb] = y[k] + 1;
            xb[nb] = x[k];
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(y[k] >=(1.0-rb)){  //cerca arriba
            nb+=1;
            yb[nb] = y[k] - 1;
            xb[nb] = x[k];
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(x[k]<=rb and y[k]<=rb){  // cerca abajo izquierda
            nb+=1;
            xb[nb] = x[k] + 1.0;
            yb[nb] = y[k] + 1.0;
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(x[k]>=(1.0-rb) and y[k]<=rb){  // cerca abajo derecha 
            nb+=1;
            xb[nb] = x[k] - 1.0;
            yb[nb] = y[k] + 1.0;
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(x[k]<=rb and y[k]>=(1.0-rb)){  // cerca arriba izquierda
            nb+=1;
            xb[nb] = x[k] + 1.0;
            yb[nb] = y[k] - 1.0;
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
        if(x[k]>=(1.0-rb) and y[k]>=(1.0-rb)){ // cerca arriba derecha
            nb+=1;
            xb[nb] = x[k] - 1.0;
            yb[nb] = y[k] - 1.0;
            vxb[nb] = vx[k];
            vyb[nb] = vy[k];
        }
    }
}

void force(vector<double> x,vector<double> y,vector<double> vx,vector<double> vy){

default_random_engine generator;
uniform_real_distribution<double> distribution(-1.0,1.0);

    for(int j=0;j<N;j++){
        double repx = 0.0;
        double repy = 0.0;
        double flockx = 0.0;
        double flocky = 0.0;
        double nflock = 0.0;

        for(int k=0;k<nb;k++){
            double d2 = pow((xb[k]-x[j]),2) + pow((yb[k]-y[j]),2);
            if((d2<=pow(rf,2)) && (j!=k)){
                flockx += vxb[k];
                flocky += vyb[k];
                nflock++;
                if(d2<=4.0*pow(r0,2)){
                    double d = sqrt(d2);
                    repx += eps*pow((1.0-d/(2.0*r0)),1.5) * (x[j]-xb[k])/d;
                    repy += eps*pow((1.0-d/(2.0*r0)),1.5) * (y[j]-yb[k])/d;
                }
            }
        }
        double normflock=sqrt(pow(flockx,2) + pow(flocky,2));
        if(nflock==0){
            normflock =1;
        } 

        flockx = alpha[j]*flockx/normflock;
        flocky = alpha[j]*flocky/normflock;
        double vnorm=sqrt(pow(vx[j],2) + pow(vy[j],2));
        double fpropx = mu*(v0[j]-vnorm)*(vx[j]/vnorm);
        double fpropy = mu*(v0[j]-vnorm)*(vy[j]/vnorm);
        double frandx = ramp[j]*distribution(generator);
        double frandy = ramp[j]*distribution(generator);
        fx[j] = (flockx + frandx + fpropx + repx);
        fy[j] = (flocky + frandy + fpropy + repy);
    }
}
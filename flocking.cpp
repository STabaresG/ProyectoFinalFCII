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

int N = 342;  // numero de agentes
int nb = 0;
double ramp = 0.1;  //amplitud fuerza aleatoria
int n_iter = 1000;  // numero de iteraciones temporales
double dt = 0.01;  // paso temporal
double r0 = 0.005;  // rango de la fuerza de repulsion
double eps = 0.0;  // amplitud fuerza de repulsion
double rf = 0.1;  // rango de la fuerza de flocking
double alpha = 1.0;  // amplitud de la fuerza de flocking
double v0 = 0.0;  // "target speed"
double mu = 10.0;  // amplitud fuerza de auto-propulsion

// inicializacion posiciones y velocidades de todos los agentes
vector <double> xb(4*N,0);
vector <double> yb(4*N,0);
vector <double> vxb(4*N,0);
vector <double> vyb(4*N,0);

// inicializacion componentes de la fuerza total
vector <double> fx(N,0);
vector <double> fy(N,0);

int main(){

    // control de flujo de datos (creacion de los archivos de salida)
    ofstream myDatax;
    myDatax.open("datos_x.txt");
  if (myDatax.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

    myDatax << fixed;

    ofstream myDatay;
    myDatay.open("datos_y.txt");
  if (myDatay.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

    myDatay << fixed;
    
    ofstream myDatavx;
    myDatavx.open("datos_vx.txt");
  if (myDatavx.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }

    myDatavx << fixed;

    ofstream myDatavy;
    myDatavy.open("datos_vy.txt");
  if (myDatavy.fail())
    {
      cout << "\nEl archivo no se pudo abrir. Verifique si existe." << endl;
      exit(1);
    }
    
    myDatavy << fixed;

    // inicializacion de posiciones y velocidades agentes reales
    vector <double> x(N,0);
    vector <double> y(N,0);
    vector <double> vx(N,0);
    vector <double> vy(N,0);

    // para generar numeros aleatorios
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
        buffer(rf,x,y,vx,vy); // genera agentes del buffer
        force(x,y,vx,vy);   // actualiza las componentes de las fuerzas sobre cada agente
    
        // ecuaciones de movimiento
        for(int i=0;i<N;i++){
            vx[i] += fx[i]*dt;
            vy[i] += fy[i]*dt;
            x[i] += vx[i]*dt;
            y[i] += vy[i]*dt;
            // ecuaciones periodicidad en x,y. Para los extremos
            x[i] = fmod((1.0+x[i]),1.0);
            y[i] = fmod((1.0+y[i]),1.0);
        }

        // guardar los datos en los archivos
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
    // cerrar archivos
    myDatax.close();
    myDatay.close();
    myDatavx.close();
    myDatavy.close();

    return 0;
}

// funcion buffer
void buffer(double rb,vector<double> x,vector<double> y,vector<double> vx,vector<double> vy)
{   
    // guarda posiciones y velocidades de los agentes reales en los totales
    for (int i=0; i<N; i++)
    {
        xb[i] = x[i];
        yb[i] = y[i];
        vxb[i] = vx[i];
        vyb[i] = vy[i];
    }
    nb=N-1; 

    // verifica cercania al borde del cuadrado.
    // genera copias en el buffer
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

// funcion force
void force(vector<double> x,vector<double> y,vector<double> vx,vector<double> vy){

// para los numeros aleatorios
default_random_engine generator;
uniform_real_distribution<double> distribution(-1.0,1.0);

    // inicializa cada fuerza
    for(int j=0;j<N;j++){
        double repx = 0.0;
        double repy = 0.0;
        double flockx = 0.0;
        double flocky = 0.0;
        double nflock = 0.0;

        // aplica la fuerza de flocking y de repulsion
        for(int k=0;k<nb;k++){
            double d2 = pow((xb[k]-x[j]),2) + pow((yb[k]-y[j]),2);
            if((d2<=pow(rf,2)) && (j!=k)){
                flockx += vxb[k];
                flocky += vyb[k];
                nflock++;  // cuenta los que estan en la zona de flocking
                if(d2<=4.0*pow(r0,2)){
                    double d = sqrt(d2);
                    repx += eps*pow((1.0-d/(2.0*r0)),1.5) * (x[j]-xb[k])/d;
                    repy += eps*pow((1.0-d/(2.0*r0)),1.5) * (y[j]-yb[k])/d;
                }
            }
        }
        double normflock=sqrt(pow(flockx,2) + pow(flocky,2));

        // si no hay ninguno en la zona de flocking, se hace normflock=1
        // para evitar division por cero
        if(nflock==0){
            normflock =1;
        } 

        // componentes fuerza de flocking
        flockx = alpha*flockx/normflock;
        flocky = alpha*flocky/normflock;

        double vnorm=sqrt(pow(vx[j],2) + pow(vy[j],2));

        // componentes de la fuerza de auto-propulsion
        double fpropx = mu*(v0-vnorm)*(vx[j]/vnorm);
        double fpropy = mu*(v0-vnorm)*(vy[j]/vnorm);

        // componentes de la fuerza aleatoria
        double frandx = ramp*distribution(generator);
        double frandy = ramp*distribution(generator);

        // guarda las componenetes de las fuerzas en la fuerza neta
        fx[j] = (flockx + frandx + fpropx + repx);
        fy[j] = (flocky + frandy + fpropy + repy);
    }
}
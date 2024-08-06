#pragma once
#include<iostream>
#include<vector>
#include<functional>
#include<math.h>
#include<algorithm>
inline float find_k(float m, float omega, float Y_0,float xi = 1.0) {  // Assume c is chosen such that xi = 1 for simplicity
    float k_lower_bound = 0;
    float k_upper_bound = 1000.0;
    float k = k_lower_bound;
    
    while (k_upper_bound - k_lower_bound > 1e-3) {
        k = (k_upper_bound + k_lower_bound) / 2;
        float omega_0 = std::sqrt(k / m);
        float omega_bar = omega / omega_0;
        float A = Y_0 * (omega_bar * omega_bar) / std::sqrt(std::pow(1 - omega_bar * omega_bar, 2) + std::pow(2 * xi * omega_bar, 2));
        
        if (A < 10.0 / 1000.0) {
            k_upper_bound = k;
        } else {
            k_lower_bound = k;
        }
    }
    
    return k;
}

/**
 * @brief quadSprings equal function,t is the time of vibration,and func is the funciton of vibration 
 *         
 * @param t ,y_0,omega.Y_0,func;
 * @return std::vector<float> 
 */
inline float quadSprings(float t,float omega,float Y_0){
    float k,c,m;//k is stiffness, c is damping, and 𝑚 is the mass of the robot body."
    m=560.0/1000;
    // k=find_k(m,omega,Y_0);
    k=11;
    cout<<"k:"<<k<<endl;
    float xi,omega_0,omega_bar,alpha;
    omega_0=sqrt(k/m);
    c=2*m*omega_0;
    xi=c/(2*m*omega_0);
    xi=1;//replace
    omega_bar=omega/omega_0;
    alpha=acos(1/sqrt(1+pow(2*xi*omega_bar,2)));
    float x; //x is the robot positon of z;
    float A=Y_0*sqrt((1+pow((2*xi*omega_bar),2))/(pow((1-omega_bar),2)+pow((2*xi*omega_bar),2)));
    float theta = acos((1-pow(omega_bar,2))/sqrt(pow((1-omega_bar),2)+pow((2*xi*omega_bar),2)))-alpha;
    x = A*sin(omega*t-theta); // robot absolute displacement 

    float z;// robot relative displacement
    A=Y_0*(pow(omega_bar,2))/sqrt(pow((1-omega_bar),2)+pow((2*xi*omega_bar),2));
    theta = acos((1-pow(omega_bar,2))/sqrt(pow((1-omega_bar),2)+pow((2*xi*omega_bar),2)));
    z=A*sin(omega*t-theta-PI/2);
    return z;

}


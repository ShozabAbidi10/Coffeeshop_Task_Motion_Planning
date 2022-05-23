    /*     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <typeinfo>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>

#include "armadillo"
#include <initializer_list>

using namespace std;
using namespace arma;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){

}

VisitSolver::~VisitSolver(){

}

void VisitSolver::loadSolver(string *parameters, int n){
  starting_position = "r0";
  string Paramers = parameters[0];  

  // Indirect variables: Whatever compuations we do, those values are stored in these
  // indirect variable.
  char const *x[]={"dummy"};   

  // Direct variables: These are the normal PDDL variable that just stored values.                              
  char const *y[]={"act-cost","triggered"}; 
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  // We are loading the way point file in the planner.
  string waypoint_file = "/root/ai4ro2/visits_domains_shared/waypoint.txt";
  parseWaypoint(waypoint_file);

  // We are loading the landmark file in the planner.
  string landmark_file = "/root/ai4ro2/visits_domains_shared/landmark.txt";
  parseLandmark(landmark_file);

  startEKF();
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;


  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);

      // This is the triggered function from which we our communicating with planner
      if(function=="triggered"){      
        trigger[arg] = value>0?1:0;
        if (value>0){
          
          // Here we are storing these strings "from" and "to" are regions, need to extract wps (poses)
          string from = tmp.substr(0,2);   
          string to = tmp.substr(3,2);
          
          act_cost = localize(from, to);
        
        }
      }
    } 
    else{
      if(function=="dummy")
      {
        dummy = value;
      }
      else if(function=="act-cost")
      {
        act_cost = value;
      }
    }
  }

  double results =  calculateExtern(dummy, act_cost);
  
  if (ExternalSolver::verbose)
  {
    cout << "(dummy) " << results << endl;
  }
  
  toReturn["(dummy)"] = results;
  return toReturn;
}

list<string> VisitSolver::getParameters(){ 
  return affected;
}

list<string> VisitSolver::getDependencies(){
  return dependencies;
}

void VisitSolver::parseParameters(string parameters){
  int curr, next;
  string line;
  ifstream parametersFile(parameters.c_str());
  if (parametersFile.is_open()){
    while (getline(parametersFile,line)){
      curr=line.find(" ");
      string region_name = line.substr(0,curr).c_str();
      curr=curr+1;
      while(true )
      {
        next=line.find(" ",curr);
        region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
        if (next ==-1)
          break;
          curr=next+1;
      }                
    }
  }
}

double VisitSolver::calculateExtern(double external, double total_cost)
{
  double cost = total_cost;;
  return cost;
}

void VisitSolver::parseWaypoint(string waypoint_file)
{  
  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(waypoint_file);

  if (parametersFile.is_open())
  {
    while (getline(parametersFile,line))
    {
      curr=line.find("[");

      string waypoint_name = line.substr(0,curr).c_str();

      curr=curr+1;
      next=line.find(",",curr);

      pose1 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; 
      next=line.find(",",curr);

      pose2 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; 
      next=line.find("]",curr);
      
      pose3 = (double)atof(line.substr(curr,next-curr).c_str());

      waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
    }
  }
}

void VisitSolver::parseLandmark(string landmark_file)
{
  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(landmark_file);
  if (parametersFile.is_open())
  {
    while (getline(parametersFile,line))
    {
      
      curr=line.find("[");
      string landmark_name = line.substr(0,curr).c_str();
      curr=curr+1;
      next=line.find(",",curr);
      pose1 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find(",",curr);
      pose2 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find("]",curr);
      pose3 = (double)atof(line.substr(curr,next-curr).c_str());
      landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
    }
  } 
}

void VisitSolver::startEKF()
{
    // Initial uncerntainty covariance
    P_prev = 
            { 
              {0.02,0,0}, 
              {0,0.02,0}, 
              {0,0,0.02} 
            };

    // The uncertainity (noise) that you gain due to the noise in the motion step itself.
    R_t = 
          { 
            {0.02, 0, 0},
            {0, 0.02, 0},
            {0, 0, 0.02} 
          };

    // The uncertainity (noise) 
    Q_t = 
          { 
            {0.01, 0},
            {0, 0.01},
          };
}

double VisitSolver::calculate_dist(double x1, double x2, double y1, double y2)
{
  return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}

double VisitSolver::calculate_angle(double x1, double x2, double y1, double y2)
{
   return atan2(y2 - y1, x2 - x1);
}

double VisitSolver::localize( string from, string to)
{

  //Starting position x and y: (x1, y1)
  double x1 = waypoint[region_mapping[from][0]][0];
  double y1 = waypoint[region_mapping[from][0]][1];

  // Goal position x and y: (x2, y2)
  double x2 = waypoint[region_mapping[to][0]][0];
  double y2 = waypoint[region_mapping[to][0]][1];

  //Calculating dist between "start" and "goal".
  double dist = calculate_dist(x1,x2,y1,y2);
  cout << endl << endl << "Cost due to distance travelled = " << dist << endl;
  
  //Calculating orientation angle between "start" and "goal".
  double angle = calculate_angle(x1,x2,y1,y2);
  cout<< "Angle between 'to' and 'from' = " << angle <<endl;

  // Odometry equations
  double x_t = x1, y_t = y1, theta_t = angle;

  
  double nos = ceil(dist/delta);

  arma::mat P_t; 

  for(int i = 0; i < nos; i++)
  { 
    // Prediction phase
    x_t += delta*cos(theta_t);
    y_t += delta*sin(theta_t);

    // matrix G_t is the Jacobian of motion model w.r.t the state x
    G_t = 
        { 
          {1, 0, 0},
          {0, 1, 0},
          {-dist*sin(theta_t), dist*cos(theta_t), 1}
        };

    P_t = G_t * P_prev * G_t.t() + R_t;

    for (auto l : landmark)
    {
      double l_x = l.second[0];
      double l_y = l.second[1];
      double q = calculate_dist(x_t,l_x,y_t,l_y);
      
      //  Measurement/Obervation Phase.
      // In every iteration all four landmark positions are measured. 
      if (l_x != 0 || l_y != 0)
      {
        arma::mat H_t =
            {
              { -(l_x - x_t) / q, -(l_y - y_t) / q, 0 },
              { (l_y - y_t) / pow(q,2), -(l_x - x_t) / pow(q,2), -1 }
            };

        arma::mat S_t = (H_t * P_t * H_t.t() + Q_t);
     
        // Kalman gain K_t
        arma::mat K_t = P_t * H_t.t() * S_t.i();

        //Updating P_t value
        P_t = (arma::eye(3,3) - K_t * H_t) * P_t;

      }
    }
  }

  double cost_due_to_uncern = trace(P_t);
  cout << "Cost due to uncertainity = " << cost_due_to_uncern << endl << endl;

  return dist + cost_due_to_uncern;
} 







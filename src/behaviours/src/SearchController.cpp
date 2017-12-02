#include "SearchController.h"
#include <angles/angles.h>
#include <cmath>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  minDist = 1.0;
  maxDist = 2.0;

  trialsPerSquareMeter = 3.0;

  numTrialsCur = 0;
  numTrialsMax = 0;

  maxAttempts = 10;

  nextFibonacci = 1;
  prevFibonacci = 1;
  waypointCounter = 0;
  
}

int SearchController::GetNextFibonacci(int prevFibonacci, int currentFibonacci)
{
  return prevFibonacci + currentFibonacci;
}

void SearchController::Reset() {
  result.reset = false;
}

double SearchController::GetAnnularArea(double minDist, double maxDist)
{
  return M_PI * maxDist * maxDist - M_PI * minDist * minDist;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  double candidateTheta;
  double candidateX;
  double candidateY;
  
  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
      waypointCounter++;
      std::cout << "waypointCounter= " << waypointCounter << "!" << std::endl;

    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
      std::cout << "First: way point: searchLocation.theta= " << searchLocation.theta << "!" << std::endl;
    }
    else
    {
      if(explorer){ //Perform spiral search

        
        if(waypointCounter == nextFibonacci){

          float distanceToCenter = hypot(currentLocation.x - centerLocation.x, currentLocation.y - centerLocation.y);

          //Get next number of fibonacci sequence
          int temp = nextFibonacci;
          nextFibonacci = GetNextFibonacci(prevFibonacci,nextFibonacci);
          prevFibonacci = temp;

          // float deltaTheta = 1<<(int) distanceToCenter;
          deltaTheta = 90/nextFibonacci;
          //convert to radians
          deltaTheta *= 0.01745;
        }

          candidateTheta = currentLocation.theta + deltaTheta;
          candidateX = currentLocation.x + (0.5 * cos(candidateTheta));
          candidateY = currentLocation.y + (0.5 * sin(candidateTheta));


      }else{ //Search near collection zone

        double dist;
        int numAttempts = 0;

        //Simply have the robot move 50 cm in a direction skewed toward its
        //current, provided that the new point lies on the current annulus.

        do
        {
          candidateTheta = rng->gaussian(currentLocation.theta, 0.785398);

          candidateX = currentLocation.x + (0.5 * cos(candidateTheta));
          candidateY = currentLocation.y + (0.5 * sin(candidateTheta));

          dist = hypot(centerLocation.x - candidateX, centerLocation.y - candidateY);

          numAttempts++;
        }
        while ((dist < minDist || dist > maxDist) && numAttempts < maxAttempts);

        // Our robot is unable to choose a valid point from its current location.
        // Move to a new point chosen at random on the current annulus.
        if (numAttempts == maxAttempts)
        {

          do
          {
            candidateTheta = rng->uniformReal(0.0, M_PI/2);
            candidateX = currentLocation.x + (0.5 * cos(candidateTheta));
            candidateY = currentLocation.y + (0.5 * sin(candidateTheta));

            dist = hypot(centerLocation.x - candidateX, centerLocation.y - candidateY);

          }
          while (dist < minDist || dist > maxDist);
        }

      }

      searchLocation.theta = candidateTheta;
      searchLocation.x = candidateX;
      searchLocation.y = candidateY;
      numTrialsCur++;
      
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}

void SearchController::SetID(int robot_id)
{
  this->robot_id = robot_id;
  std::cout << "I have ID " << this->robot_id << "!" << std::endl;

  if(robot_id > 0){
    explorer = true;
    std::cout << "I am an explorer!" << std::endl;
  }
}

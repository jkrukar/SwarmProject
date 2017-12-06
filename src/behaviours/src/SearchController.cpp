#include "SearchController.h"
#include <angles/angles.h>

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

  // Variables for inactive code
  // minDist = 1.0;
  // maxDist = 2.0;
  // numTrialsCur = 0;
  // numTrialsMax = 0;
  // maxAttempts = 10;
  // explorer = false;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.10) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 2) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 2 || attemptCount == 0) 
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

      a = (robot_id + 1) * 8 * M_PI + M_PI / 8;
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      /*searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));*/

      // Sample points along an Archimedian Spiral - exactly eight points per
      // winding UNLESS the distance of this point from the center exceeds 7.5
      // meters, in which case return to sampling points near the center (around
      // where the first robot begins by default).

      if (b * (M_PI / 4 * k + a) > 7.5)
      {
        a = 8 * M_PI + M_PI / 8;
        k = 0;
      }

      searchLocation.x = centerLocation.x + b * (M_PI / 4 * k + a) * cos(M_PI / 4 * k + a);
      searchLocation.y = centerLocation.y + b * (M_PI / 4 * k + a) * sin(M_PI / 4 * k + a);
     
      k++;
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }
// 13
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
}

int SearchController::GetID()
{
  return this->robot_id;
}

bool SearchController::GetExploreState(){
  // if(explorer){
  //   return true;
  // }
  return false;
}

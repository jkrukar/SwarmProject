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
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements the spiral search algorithm with site fidelity.
 */
Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.10) {
      attemptCount = 0;

      if(hasClusterLocation){
          hasClusterLocation = false; //Reset cluster flag once back at cluster
      }
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

      // Assign each robot a certain initial depth in the spiral - with robot 0
      // beginning closest to the center, and robot 2 beginning furthest from
      // the center.  Also, have each robot choose an initial point that doesn't
      // require them to cross the collection zone so that they don't run into
      // each other on startup.

      if (currentLocation.y > centerLocation.y + 0.2)
      {
        // The y-location of this robot is greater than the center location.
        // Have it choose an initial point on the spiral with a positive y (at
        // roughly an angle of 5 * pi / 8 radians with the horizontal).

        a = (robot_id + 1) * 8 * M_PI + 5 * M_PI / 8;
      }
      else if (currentLocation.y < centerLocation.y - 0.2)
      {
        // The y-location of this robot is less than the center location.  Have
        // it choose an initial point on the spiral with a negative y (at
        // roughly an angle of -3 * pi / 8 radians with the horizontal).

        a = (robot_id + 1) * 8 * M_PI - 3 * M_PI / 8;
      }
      else if (currentLocation.x > centerLocation.x + 0.2)
      {
        // The x-location of this robot is greater than the center location.
        // Have it choose an initial point on the spiral with a positive x (at
        // roughly an angle of pi / 8 radians with the horizontal).
        
        a = (robot_id + 1) * 8 * M_PI + M_PI / 8;
      }
      else
      {
        // Either the x-location of this robot is less than the center location,
        // or we've missed every case somehow.  Either way, have it choose an
        // initial point on the spiral with a negative x (at roughly an angle of
        // -7 * pi / 8 radians with the horizontal).

        a = (robot_id + 1) * 8 * M_PI - 7 * M_PI / 8;
      }
    }
    else
    {
      // Sample points along an Archimedian Spiral - exactly eight points per
      // winding UNLESS the distance of this point from the center exceeds 6
      // meters (for a 12 meter by 12 meter arena), in which case return to
      // sampling points near the center (around where the first robot begins by
      // default).

      if (b * (M_PI / 4 * k + a) > 6)
      {
        a = 8 * M_PI + M_PI / 8;
        k = 0;
      }

      searchLocation.x = centerLocation.x + b * (M_PI / 4 * k + a) * cos(M_PI / 4 * k + a);
      searchLocation.y = centerLocation.y + b * (M_PI / 4 * k + a) * sin(M_PI / 4 * k + a);
     
      k++;
    } 

    // Return to cluster location, if a cluster was seen previously.
    if(hasClusterLocation){
      std::cout << "Heading back to cluster" << std::endl;
      searchLocation.x = clusterLocation.x;
      searchLocation.y = clusterLocation.y;
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
}

int SearchController::GetID()
{
  return this->robot_id;
}

bool SearchController::GetExploreState(){
  return false;
}

Point SearchController::GetClusterLocation(){
  return clusterLocation;
}
 
void SearchController::SetClusterLocation(Point clusterLocation){
  this->clusterLocation = clusterLocation;
}

bool SearchController::GetClusterLocationState(){
  return hasClusterLocation;

}

void SearchController::SetClusterLocationState(bool newState){
  hasClusterLocation = newState;
}

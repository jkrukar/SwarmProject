#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  // Minimum distance from the center at which a waypoint may be placed.
  double minDist;
  // Maximum distance from the center at which a waypoint may be placed.
  double maxDist;
  // Amount of extra distance to add to the minDist and maxDist after an annulus
  // has been searched for the required number of times.
  double addlDist;

  // Number of waypoints that will be chosen in each annular region before a new
  // one is selected for each square meter of area in the current annulus.
  double trialsPerSquareMeter;

  int numTrialsCur;
  int numTrialsMax;

  int maxAttempts;

  double GetAnnularArea(double minDist, double maxDist);
};

#endif /* SEARCH_CONTROLLER */

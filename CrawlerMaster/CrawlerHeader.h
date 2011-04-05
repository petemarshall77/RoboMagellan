//
// CrawlerHeader.h
//
//   This header file contains definitions and data structures common to different RoboMagellan controllers
//

// Define Wire Library Addresses
#define WIRE_ADDRESS_NAVIGATION  3
#define WIRE_ADDRESS_SENSORS     5


// Structure for sending/receiving waypoints
struct WAYPOINT_STRUCT {
  double latitude;
  double longitude;
};

// Structure for sending/receiving navigation data
struct NAV_STRUCT {
  double distance;
  int    angle;
};

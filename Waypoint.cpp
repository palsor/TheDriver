#include "Waypoint.h"

//
// constructor
//
Waypoint::Waypoint() {
  lat = 0;
  lon = 0; 
}

//
// getters and setters
//
float Waypoint::getLat() { return lat; }
float Waypoint::getLon() { return lon; }
void Waypoint::setLocation(float newLat, float newLon) {
  lat = newLat;
  lon = newLon;
}

//
// print - print out debug output of the waypoint
//
void Waypoint::print() {
  softSerial.print("lat: ");
  softSerial.print(lat);
  softSerial.print("   lon: ");
  softSerial.println(lon);  
}

float Waypoint::bearingToTarget(Waypoint* target) {
  // convert to radians
  float lat1 = convDegreesToRadians(lat);
  float lon1 = convDegreesToRadians(lon);
  float lat2 = convDegreesToRadians(target->getLat());
  float lon2 = convDegreesToRadians(target->getLon());
  
  // calculate intermediate terms
  float dPhi = calcDPhi(lat1,lat2);
  float dLat = lat2-lat1;
  float dLon = calcDLon(lon1,lon2);
  float q = calcQ(dPhi, dLat, lat1);
  
  //  	θ = atan2(Δlon, Δφ) 	 
  float gpsDestinationBearingRadians = atan2(dLon,dPhi);  
  float gpsDestinationBearingDegrees = gpsDestinationBearingRadians * 180 / 3.14159265358979323846;
  
  // Hack to fix the math (empirically added)
  if(gpsDestinationBearingDegrees < 0) {
    gpsDestinationBearingDegrees += 360;
  }
  return(gpsDestinationBearingDegrees);
} 

//
// distanceToTarget - determines the distance from this waypoint to the target waypoint
//
float Waypoint::distanceToTarget(Waypoint* target) {
  
  // fill in the code calculate the distance between two lat/long combos, target->getLat() and target->getLon() will return the target data

  // convert to radians
  float lat1 = convDegreesToRadians(lat);
  float lon1 = convDegreesToRadians(lon);
  float lat2 = convDegreesToRadians(target->getLat());
  float lon2 = convDegreesToRadians(target->getLon());
  
  // calculate intermediate terms  float dPhi = calcDPhi(lat1,lat2);
  float dPhi = calcDPhi(lat1,lat2);
  float dLat = lat2-lat1;
  float dLon = calcDLon(lon1,lon2);
  float q = calcQ(dPhi, dLat, lat1);
    
  //  	d = √(Δlat² + q².Δlon²).R 	[pythagoras] 
  float gpsDistanceToDestination = sqrt(dLat*dLat + q*q*dLon*dLon) * 6371;
  return(gpsDistanceToDestination);
}

float Waypoint::convDegreesToRadians(float degree) {
  return(degree * 3.14159265358979323846 / 180);
}

float Waypoint::calcDPhi(float lat1, float lat2) {
  // Δφ = ln(tan(lat2/2+π/4)/tan(lat1/2+π/4)) 	[= the ‘stretched’ latitude difference]
  return(log(tan(lat2/2 + 0.78539816339744830962)/tan(lat1/2 + 0.78539816339744830962)));
}

float Waypoint::calcDLon(float lon1, float lon2) {
  float dLon = lon2-lon1;

  //  if (Math.abs(dLon) > Math.PI) {
  //    dLon = dLon>0 ? -(2*Math.PI-dLon) : (2*Math.PI+dLon);
  //  }
// COMPILE ERROR: abs function not linked correctly (indicates abs as variable not declared in this scope)
//  if(abs(dLon) > 3.14159265358979323846) {
//    if(dLon > 0) {
//      dLon = -(2 * 3.14159265358979323846 - dLon);
//    } else {
//      dLon = (2 * 3.14159265358979323846 + dLon);
//    }
//  }  
  if(dLon > 3.14159265358979323846) {
    dLon = -(2 * 3.14159265358979323846 - dLon);
  }
  if(dLon < -3.14159265358979323846) {
    dLon = (2 * 3.14159265358979323846 + dLon);
  }

  return(dLon);
}

float Waypoint::calcQ(float dPhi, float dLat, float lat1) {
  //if E:W line, 	q = cos(lat1) 	 
  //otherwise, 	q = Δlat/Δφ 	 
  float q;
  if(dPhi != 0) {
    q = dLat / dPhi;
  } else {
    q = cos(lat1);
  }
  return(q);
}

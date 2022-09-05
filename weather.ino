// ---------- weather.ino ----------
//
// This module contains functions to download and process weather data from various weather API's.

void getWeatherData() {

  // REMINDER: in C++, the compiler treats myarray as a POINTER to &myarray[0], 
  //   i.e., myarray is shorthand for &myarray[0], so myarray is a POINTER - see Practical C++ Programming, p235
  char wxserver[100];  // weather API server
  char wxpath[500];    // weather API path

  // Get latitude and longitude strings for this site, parmVal() gets the string representations
  char* sitelat = parm_site_latitude.parmVal();
  char* sitelon = parm_site_longitude.parmVal();
  //Serial << "sitelat = " << sitelat << "\n";
  //Serial << "sitelon = " << sitelon << "\n";

  // Create start and end date strings based on the current time
  time_t t = myunixtime;
  char start_date[] = "yyyy-mm-dd";
  char theyear[] = "yyyy", themonth[] = "mm", theday[] = "dd";
  sprintf(theyear, "%d", year(t)); sprintf(themonth, "%02d", month(t)); sprintf(theday, "%02d", day(t));
  strcpy(start_date, theyear); strcat(start_date, "-"); strcat(start_date, themonth); strcat(start_date, "-"); strcat(start_date, theday);
  t += 86400;  // add a day
  char end_date[] = "yyyy-mm-dd";
  sprintf(theyear, "%d", year(t)); sprintf(themonth, "%02d", month(t)); sprintf(theday, "%02d", day(t));
  strcpy(end_date, theyear); strcat(end_date, "-"); strcat(end_date, themonth); strcat(end_date, "-"); strcat(end_date, theday);
  //Serial << "start_date = " << start_date << "\n";
  //Serial << "end_date = " << end_date << "\n";


  // *** Construct various API server and path names ***
  /*
  // VisualCrossing weather API <-- ***NOT WORKING*** SSLClient throws DN mismatch error
  strcpy(wxserver, "weather.visualcrossing.com");  // ***trust_anchors.h must include this HTTPS server***
  strcpy(wxpath, "/VisualCrossingWebServices/rest/services/timeline/");
  strcat(wxpath, sitelat);
  strcat(wxpath, "%2C%20");
  strcat(wxpath, sitelon);
  strcat(wxpath, "/");
  strcat(wxpath, start_date);
  strcat(wxpath, "/");
  strcat(wxpath, end_date);
  strcat(wxpath, "?unitGroup=us&include=hours%2Calerts&key=GVGD3VPKTUGPEKVCF26XUNY4U&contentType=json");
  */

  /*
  // Open-Meteo API <-- NOT WORKING
  // server updates forecast every 3h
  strcpy(wxserver, "api.open-meteo.com");  // ***trust_anchors.h must include this HTTPS server***
  strcpy(wxpath, "/v1/forecast?latitude=");
  strcat(wxpath, sitelat);
  strcat(wxpath, "&longitude=");
  strcat(wxpath, sitelon);
  strcat(wxpath, "&hourly=windgusts_10m&windspeed_unit=ms&timeformat=unixtime&start_date=");
  strcat(wxpath, start_date);
  strcat(wxpath, "&end_date=");
  strcat(wxpath, end_date);
  */

  // NWS API <-- TESTED, WORKING
  strcpy(wxserver, "api.weather.gov");  // ***trust_anchors.h must include this HTTPS server***
  strcpy(wxpath, "/alerts/active?point=");
  strcat(wxpath, sitelat);
  strcat(wxpath, "%2C");
  strcat(wxpath, sitelon);
  getNWSAPIData(wxserver, wxpath);  // see webclient.ino
  // if NO ALERTS, ~232 bytes are returned


// *** Process the returned data ***

  


}

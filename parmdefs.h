// ---------- parmdefs.h ----------
// This module defines ALL Controller Operating Parameters
// MAX_PARMS 40 (see parms.h) current count = 37
// All parms are saved to SD and the Config Server and persist across controller resets.
//
// Any parms which should NOT be checked against those on the Config Server should be added to the udp-config.py script
//   on the Config server (e.g., /Users/WWE/Sites/WWE/bin/udp/udp-config.py)
//
// Parms MUST be listed below in the order they should appear on the Controller Operating Parameters webpage - see webserver.ino


// ***TURBINE PARMS (14)***
// Shutdown State
// 0 = Normal Operation, 1 = Shutdown (routine), 2 = Shutdown (emergency)
Parm parm_shutdown_state = Parm("shutdown_state", "Shutdown State", "0/1/2", 0);  // this is the most frequently changed parm

// Furl control
// These are thresholds ABOVE which we furl (or short) and from which there is AUTOMATIC recovery.
// NOTE: All thresholds are float parms!
Parm parm_furl_init_v = Parm("furl_init_voltage", "Furl@ V >", "V", (float)310.0);
Parm parm_furl_init_i = Parm("furl_init_current", "Furl@ I >", "A", (float)12.0);
Parm parm_furl_init_rpm = Parm("furl_init_rpm", "Furl@ RPM >", "rpm", (float)470.0);
Parm parm_furl_init_ws = Parm("furl_init_windspeed", "Furl@ WS >", "mph", (float)(35.0*MPH2MS));
Parm parm_sc_emer_ws = Parm("sc_emer_windspeed", "Brake@ WS >", "mph", (float)(40.0*MPH2MS));

// Shorting contactor control
// Thresholds ABOVE which the SC shorts and from which there is ONLY MANUAL recovery.
Parm parm_sc_furled_failsafe_v = Parm("sc_furled_failsafe_V", "Shutdown@ V >", "V", (float)320.0);
Parm parm_sc_furled_failsafe_i = Parm("sc_furled_failsafe_I", "Shutdown@ I >", "A", (float)15.0);
Parm parm_sc_furled_failsafe_rpm = Parm("sc_furled_failsafe_rpm", "Shutdown@ RPM >", "rpm", (float)520.0);
Parm parm_sc_failsafe_tp = Parm("sc_failsafe_tp", "Shutdown@ TP >", "deg", (float)98.0);

// Thresholds BELOW which the SC can safely short.
Parm parm_sc_exer_v = Parm("sc_exer_voltage", "Brake@ V <", "V", (float)150.0);
Parm parm_sc_exer_i = Parm("sc_exer_current", "Brake@ I <", "A", (float)1.5);
Parm parm_sc_exer_rpm = Parm("sc_exer_rpm", "Brake@ RPM <", "rpm", (float)180.0);

// Number of alternator pole pairs (used in calculating RPM)
Parm parm_alt_poles = Parm("alt_poles", "Alternator Poles", "", 6);  // Alxion alternator



// ***NETWORK AND SERVER PARMS (10)***
// Dis/allow the controller to override parm vals if they differ from those saved on the Config Server
Parm parm_ovrd = Parm("ovrd", "Override Updates?", "0/1", 0);  // default = 0 = don't override

// Wait time between controller config requests to the Update Server
Parm parm_cfg_minutes = Parm("cfg_minutes", "Update Interval", "mins", 5);

// Update Server IP address. This server handles FIRMWARE updates - see doFWUpdate() in wwe.ino
char pbuf20[20] = "192.168.1.4";
Parm parm_cfg_ip = Parm("cfg_ip", "Update Server", pbuf20);

// Update Server port which handles FIRMWARE updates - see doFWUpdate() in wwe.ino
Parm parm_cfg_port = Parm("cfg_port", "Update Port", "", 49152);

// Name of the firmware binary file currently being executed.
// NOTE: Binary file name need to be in 8.3 format for saving to SD. Filenames could be coded by date, e.g., 20210627.bin
char pbuf21[50] = "wwe.bin";
Parm parm_binfile = Parm("binary_filename", "Update File", pbuf21);

// Data Server IP address. This server handles UDP data and config requests from the controller.
char pbuf19[20] = "192.168.1.4";
Parm parm_udp_ip = Parm("udp_ip", "Data Server", pbuf19);

// Controller LAN IP address (if static) or DHCP
char pbuf23[20] = "192.168.1.40";
Parm parm_controller_ip = Parm("controller_ip", "Controller IP", "DHCP");  // "DHCP" or pbuf23

// LAN Gateway
char pbuf24[20] = "192.168.1.1";
Parm parm_gateway_ip = Parm("gateway_ip", "Gateway IP", pbuf24);

 // LAN DNS
Parm parm_dns1_ip = Parm("dns1_ip", "DNS1 IP", pbuf24);

// Nuvation BMS IP address (via Modbus/TCP)
char pbuf22[20] = "192.168.1.21";
//Parm parm_nuvation_ip = Parm("nuvation_ip", "Nuvation IP", pbuf22);  // Nuvation device default = 192.168.1.21
Parm parm_nuvation_ip = Parm("nuvation_ip", "Nuvation IP", IPAddress(192,168,1,21));  // Nuvation device default = 192.168.1.21



// ***HVDL PARMS (5)***
// These parms are used by manageDumpLoad() - see furlctl.ino
Parm parm_hvdl_active = Parm("hvdl_active", "HVDL Active", "0/1", 0);
Parm parm_hvdl_R = Parm("hvdl_R", "HVDL Resistance", "Ohms", 18);
Parm parm_hvdl_Vstart = Parm("hvdl_Vstart", "HVDL Vstart", "V", 120);
Parm parm_hvdl_Vspan = Parm("hvdl_Vspan", "HVDL Vspan", "V", 180);
Parm parm_hvdl_Pmax = Parm("hvdl_Pmax", "HVDL Pmax", "W", 2200);



// ***LOCAL PARMS (7)***
// Disconnect switches for associated PV arrays (via Modbus/RTU)
Parm parm_PV1_disc = Parm("pv1_disc", "PV1 Disconnect", "0/1", 0);  // PV1, TS-MPPT-30 controller
Parm parm_PV2_disc = Parm("pv2_disc", "PV2 Disconnect", "0/1", 0);  // PV2, TS-MPPT-60 controller

// Site latitude and longitude parms, used for weather API queries
Parm parm_site_latitude = Parm("site_latitude", "Site Latitude", "42.2534");
Parm parm_site_longitude = Parm("site_longitude", "Site Longitude", "-76.5702");

// Local time zone offset from UTC. No DST adjustment!
Parm parm_TZ_offset = Parm("TZ_offset", "Local Time - UTC", "hrs", -5);  // EST-UTC = -5 (default), EDT-UTC = -4

// Controller restart count parm
Parm parm_num_resets = Parm("num_resets", "Restart Count", "", 0);  // default = none

// Weather API furl trigger override (see furlctl.ino)
Parm parm_wx_override = Parm("wx_override", "WX Override?", "0/1", 0);  // default = 0 = don't override



// ***UNUSED PARMS***
// Morningstar IP parms - UNUSED because Modbus/TCP turns out to be SLOWER than Modbus/RTU
//char pbuf25[20] = "192.168.1.ddd";
//Parm parm_mppt600_1_ip = Parm("mppt600_1_ip", "MPPT600_1 IP", pbuf25);
//char pbuf26[20] = "192.168.1.ddd";
//Parm parm_mppt60_1_ip = Parm("mppt60_1_ip", "MPPT60_1 IP", pbuf26);

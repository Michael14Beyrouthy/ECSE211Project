package ca.mcgill.ecse211.project;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * WifiInfo class, used to connect robot to a server and pass it parameters
 * @author micha
 *
 */
public class WifiInfo {
/*
 * The server IP to connect to
 * During the demo and final demo, it is "192.168.2.3"
 * When we test, it is the laptop's IP assigned by the DPM router (get from network settings on laptop)
 */
 private static final String SERVER_IP = "192.168.2.22";
 /*
  * Our team number
  */
 private static final int TEAM_NUMBER = 5;

/*
 * Enable/diable printing of debug info from Wifi class
 */
 private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
 
 /*
  * An int array to store the parameters
  */
 
 public static long Corner;
 
 public static int Tunnel_LL_x;
 public static int Tunnel_LL_y;
 public static int Tunnel_UR_x;
 public static int Tunnel_UR_y;

 public static int Search_LL_x;
 public static int Search_LL_y;
 public static int Search_UR_x;
 public static int Search_UR_y;

 public static int Start_LL_x;
 public static int Start_LL_y;
 public static int Start_UR_x;
 public static int Start_UR_y; 
 
 public static int Island_LL_x;
 public static int Island_LL_y;
 public static int Island_UR_x;
 public static int Island_UR_y;
 
 
 
 /**
  * Class Constructor
  */
 public WifiInfo() {
   
 }
 
 public void getInfo() {
// Initialize WifiConnection class
   WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
   try {
     Map data = conn.getData();
     
     if(((Long) data.get("RedTeam")).intValue()==TEAM_NUMBER) { //if our team is the red team
       Corner=((Long) data.get("RedCorner")).intValue(); //starting corner
       
       Start_LL_x=((Long) data.get("Red_LL_x")).intValue(); //lower left x of base region
       Start_LL_y=((Long) data.get("Red_LL_y")).intValue(); //lower left y of base region
       Start_UR_x=((Long) data.get("Red_UR_x")).intValue(); //Upper right x of base region
       Start_UR_y=((Long) data.get("Red_UR_y")).intValue(); //Upper right y of base region
       
       Island_LL_x=((Long) data.get("Island_LL_x")).intValue(); //lower left x of island
       Island_LL_y=((Long) data.get("Island_LL_y")).intValue(); //lower left y of island
       Island_UR_x=((Long) data.get("Island_UR_x")).intValue(); //Upper right x of island
       Island_UR_y=((Long) data.get("Island_UR_y")).intValue(); //Upper right y of island
       
       Tunnel_LL_x=((Long) data.get("TNR_LL_x")).intValue(); //lower left x of tunnel
       Tunnel_LL_y=((Long) data.get("TNR_LL_y")).intValue(); //lower left y of tunnel
       Tunnel_UR_x=((Long) data.get("TNR_UR_x")).intValue(); //Upper right x of tunnel
       Tunnel_UR_y=((Long) data.get("TNR_UR_y")).intValue(); //Upper right y of tunnel
       
       Search_LL_x=((Long) data.get("SZR_LL_x")).intValue(); //lower left x of search zone
       Search_LL_y=((Long) data.get("SZR_LL_y")).intValue(); //lower left y of search zone
       Search_UR_x=((Long) data.get("SZR_UR_x")).intValue(); //Upper right x of search zone
       Search_UR_y=((Long) data.get("SZR_UR_y")).intValue(); //Upper right y of search zone
       
       
     }
     else if(((Long) data.get("GreenTeam")).intValue()==TEAM_NUMBER) { //if our team is the green team
       Corner=((Long) data.get("GreenCorner")).intValue();
       
       Start_LL_x=((Long) data.get("Green_LL_x")).intValue(); //lower left x of base region
       Start_LL_y=((Long) data.get("Green_LL_y")).intValue(); //lower left y of base region
       Start_UR_x=((Long) data.get("Green_UR_x")).intValue(); //Upper right x of base region
       Start_UR_y=((Long) data.get("Green_UR_y")).intValue(); //Upper right y of base region
       
       Island_LL_x=((Long) data.get("Island_LL_x")).intValue(); //lower left x of island
       Island_LL_y=((Long) data.get("Island_LL_y")).intValue(); //lower left y of island
       Island_UR_x=((Long) data.get("Island_UR_x")).intValue(); //Upper right x of island
       Island_UR_y=((Long) data.get("Island_UR_y")).intValue(); //Upper right y of island
       
       Tunnel_LL_x=((Long) data.get("TNG_LL_x")).intValue(); //lower left x of tunnel
       Tunnel_LL_y=((Long) data.get("TNG_LL_y")).intValue(); //lower left y of tunnel
       Tunnel_UR_x=((Long) data.get("TNG_UR_x")).intValue(); //Upper right x of tunnel
       Tunnel_UR_y=((Long) data.get("TNG_UR_y")).intValue(); //Upper right y of tunnel
       
       Search_LL_x=((Long) data.get("SZG_LL_x")).intValue(); //lower left x of search zone
       Search_LL_y=((Long) data.get("SZG_LL_y")).intValue(); //lower left y of search zone
       Search_UR_x=((Long) data.get("SZG_UR_x")).intValue(); //Upper right x of search zone
       Search_UR_y=((Long) data.get("SZG_UR_y")).intValue(); //Upper right y of search zone
     }
     else { //if our team is neither green nor red
       throw new Exception("Our team is not represented");
     }
     
   }
   catch (Exception e) {
     System.err.println("Error: " + e.getMessage());
   }
 }
 
 
}


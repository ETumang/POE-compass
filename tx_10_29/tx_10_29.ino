#include <VirtualWire.h>

void setup()
{
  vw_set_tx_pin(12);
  vw_setup(2000) ;
  time = millis();
}

void loop()
{
data = get_gps_data

process(data)

time_n = millis()
if (time - time_n > 10000);
createMessage("G", data);
time = time_n;
end
}





void createMessage(String type, float lon_or_time, float lat, float s)
{
  String message = "N";
  int time = -1;
  int lon_int = 0;
  int lat_int = 0;
  
   
  switch type:
    type "T":
    time = int(lon_or_time);
    message = type + " " + time + "E";
    break;
    
    type "G":
    lon_int = 
    lat_int = 
    s = 

    message = type + " "+lon_int+" "+ lat_int+ " " + s + "E"
    
    vw_send(message)
}
    
    

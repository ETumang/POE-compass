#include <VirtualWire.h>//uses Timer1- be aware of this!

void setup()
{
  vw_set_tx_pin(12);
  vw_setup(2000) ;
  int time = millis();
  createMessage("T", millis(),0,0);//sends time signal at beginning of game
  int scale = 10000;//number of decimal points to send
}

void loop()
{
data = get_gps_data//placeholder for actual data gathering function

process(data)//another placeholder

time_n = millis()
if (time - time_n > 5000 & fix == True);//TODO:test to see how long a battery lasts w. this transmission rate.  Can be decreased if necessary
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
    lon_int = int(lon*scale)
    lat_int = int(lat*scale)
    s = int(s)
    message = type + " "+lon_int+" "+ lat_int+ " " + s + "E";
    break;
    
    vw_send(message);
    vw_wait_tx();
}

void setup()
{
 vw_setup(2000);
 vw_set_rx_pin(12);
 byte message = [VW_MAX_MESSAGE_LEN];
}

void loop():
 if vw_get_message(message, &msgLength):
 {
   message = message[]
 


  

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Ticker.h>
#include <WiFiManager.h>
// network ettings --------------
//

char ssid[] = "PGP";
char psswd[] = "oasis2018";
IPAddress ip(192, 168, 1, 116);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WiFiUDP Udp;

// ip add list: 145, 133, 165, 186
const IPAddress outIp(192,168,1,133);
const IPAddress outIp1(192,168,1,145);
const IPAddress outIp2(192,168,1,165);
const IPAddress outIp3(192,168,1,186);
const unsigned int outPort = 4001;
const unsigned int localPort = 4001;

// osc settings
OSCErrorCode error;

// microwave sensor settings
#define interrupPin D2
#define statePin D4 
volatile byte interruptCounter;
int numberOfInterrupts = 0;
volatile int sensor_state = LOW;

// declare functions
void handle_movement();

// ticker
Ticker handleMicrowave(handle_movement, 1000);
// led strip vars
float wichStrip;


void setup(){

  Serial.begin(115200);
  pinMode(statePin, OUTPUT);
  pinMode(interrupPin, INPUT_PULLUP);
  /// interrupt and ticker settings 
  attachInterrupt(digitalPinToInterrupt(interrupPin), handleInterrupts, FALLING);
  handleMicrowave.start();


  // Static IP Setup Info Here...
//  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid,psswd);

  while(WiFi.status() != WL_CONNECTED) {

    delay(500);
    Serial.print(".");

  }

  Serial.println("wifi connected");
  Serial.println(WiFi.localIP());
  Serial.println("starting udp");
  Udp.begin(localPort);


}
//--- interrupt handler
void handleInterrupts () {
  interruptCounter++;
}
//---------
void handle_movement () {

//Serial.println("ticker callback");
  // if the state changed more than once, it means there is a moving object
  // can be adjusted on the situation, equivalent to adjust treshhold of detection speed
  // for moving object
  if (interruptCounter > 1) {

    numberOfInterrupts ++;
    sensor_state = HIGH;
    interruptCounter = 0; // reset de interrupt counter
    wichStrip = random(3);
    yield();

  }else {

    interruptCounter = 0; // if it has not reached the treshold
   sensor_state = LOW;
  }

}


//--------- osc message listener 
void listen_osc_messages(OSCMessage &inc_msg) {

  int value = inc_msg.getInt(0);
}
//-----------
void loop() {

//Serial.println(interruptCounter);
  handleMicrowave.update();
  digitalWrite(statePin, sensor_state);
  /*  digitalWrite(interrupPin, HIGH);
      delay(500);
      digitalWrite(interrupPin, LOW);
      delay(500);*/



if(sensor_state == HIGH) {

  OSCMessage msg("/gesto/1");
  msg.add(sensor_state);

      Udp.beginPacket(outIp, outPort);
      msg.send(Udp);
      Udp.endPacket(); 

      Udp.beginPacket(outIp1, outPort);
      msg.send(Udp);
      Udp.endPacket();
      
    
      Udp.beginPacket(outIp2, outPort);
      msg.send(Udp);
      Udp.endPacket();
      
    
      Udp.beginPacket(outIp3, outPort);
      msg.send(Udp);
      Udp.endPacket();

      

      msg.empty();
  
}
  //  msg.add(my_data_handler.test.at(2));
  /*
     msg.add(my_data_handler.getData());
     msg.add(my_data_handler.rec_gesture_1);
     msg.add(my_data_handler.gesturePoints[0][1]);
     msg.add(my_data_handler.gestureDist[0]);
     Udp.beginPacket(outIp, outPort);
     msg.send(Udp);
     Udp.endPacket();
     msg.empty(); */
  //--------------
  //digitalWrite(2, HIGH);

  // incoming osc messages are dispatched 
  // to the listener
  OSCMessage inc_msg;
  int sizeOfmsg = Udp.parsePacket();

  if (sizeOfmsg > 0) {

    while (sizeOfmsg--) {


      inc_msg.fill(Udp.read());
    }
    if (!inc_msg.hasError()) {
     inc_msg.dispatch("/gesto/1",listen_osc_messages );
    } else {
      error = inc_msg.getError();
      /*      Serial.print("error: ");
              Serial.println(error);*/
    }
  }


/*   Serial.print("number of interrupts  ");
    Serial.print(interruptCounter);
    Serial.print("sensor_state  ");
    Serial.println(sensor_state);*/
  yield();
}




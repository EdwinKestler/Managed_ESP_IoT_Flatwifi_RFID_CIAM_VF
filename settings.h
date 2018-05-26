
#include <WString.h>                // include the String library
#include <IPAddress.h>              // include the IPAdress library
//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "p03mmr"
#define DEVICE_TYPE "ESP"
#define DEVICE_ID "CIAM"
#define TOKEN "hY5OOupZk*U1yMl1G8"
//-------- Customise the above values --------

#define internetS   "eospower.flatbox.io"
//#define Cayala      "10.130.19.250"
//#define MiraFlores  "10.130.14.240"
//#define Fraijanes   "10.130.15.245"
//#define Zona9       "10.130.12.210"
//#define Zona18      "10.130.16.245"

//-------- Customise these values-----------

char MQTTServer [] = internetS;
String FirmwareVersion= "V1.10";                                        //read in chage history
String HardwareVersion= "V2.01";                                        //read in chage history 
//---------Blurmix Topics---------------------

const char publishTopic[] = "iot-2/evt/status/fmt/json";
const char responseTopic[] ="iotdm-1/response/";
const char manageTopic[] = "iotdevice-1/mgmt/manage";
const char updateTopic[] = "iotdm-1/device/update";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/reboot";

//-----------Variables de Configuracion del Servicio de NTP
//-------- Configuracion de parametros de servicio remots de hora (NTP Servers:)
#if defined (internetS)
       IPAddress timeServer(129,  6, 15, 29); // time.nist.gov NTP ;
       const char* ntpServerName = "time.nist.gov"; //const char*;
       unsigned int localPort = 2390;  // local port to listen for UDP packets
       const int timeZone = -6;  // Eastern central Time (USA)
#else
      IPAddress timeServer(172, 20,  1,235); // time.nist.gov NTP server IPAddress timeServer(192,168,120,211);
      const char* ntpServerName = "172.20.1.235"; //const char* ntpServerName = "192.168.120.211";
      unsigned int localPort = 2390;  // local port to listen for UDP packets
      const int timeZone = -6;  // Eastern central Time (USA)
#endif


//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long UInterval     = 1000UL; //Variable configurable remotamente sobre el interbalo de publicacion

//-------- Variables de ERROR EN ENVIO de paquetes de MQTT ANTES DE REINICIO
#define FAILTRESHOLD 150
const float BATTRESHHOLD = 3.3;



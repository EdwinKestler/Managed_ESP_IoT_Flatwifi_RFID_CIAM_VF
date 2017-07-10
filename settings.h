//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "p03mmr"
#define DEVICE_TYPE "ESP"
#define DEVICE_ID "CIAM"
#define TOKEN "hY5OOupZk*U1yMl1G8"
//-------- Customise the above values --------

//-------- Customise these values-----------
//---------Blurmix Topics---------------------

const char publishTopic[] = "iot-2/evt/status/fmt/json";
const char responseTopic[] ="iotdm-1/response/13636023";
const char manageTopic[] = "iotdevice-1/mgmt/manage";
const char updateTopic[] = "iotdm-1/device/update";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/reboot";

//-----------Variables de Configuracion del Servicio de NTP
//-------- Configuracion de parametros de servicio remots de hora (NTP Servers:)

IPAddress timeServer(192,168,120,211); // time.nist.gov NTP server IPAddress timeServer(192,168,120,211);
const char* ntpServerName = "192.168.120.211"; //const char* ntpServerName = "192.168.120.211";
unsigned int localPort = 2390;  // local port to listen for UDP packets
const int timeZone = -6;  // Eastern central Time (USA)

//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long UInterval     = 1000UL; //Variable configurable remotamente sobre el interbalo de publicacion

//-------- Variables de ERROR EN ENVIO de paquetes de MQTT ANTES DE REINICIO
#define FAILTRESHOLD 150
const float BATTRESHHOLD = 3.3;


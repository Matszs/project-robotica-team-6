# Dit document beschrijft de angular tracker code. Deze zou voldoende moeten zijn om te snappen hoe het werkt.
## Angular tracker
Angular tracker krijgt input van vision in de vorm van een afstand tot het object wat gezien wordt.
Deze input is in de vorm van een x, y en een z positie, dit wordt gebruikt om de robot naar het object te laten rijden.
In deze node wordt deze waardes eerst gefilterd en daarna naar motor gepublished. 
### globals
`#include <vision/TrackedPosition.h>`
We includen hier een ander bestand. De msg file maakt via catkin make hier een header file van. De andere includes die je daarboven ziet zijn standaard dingen van ros. Deze worden gebruikt om waardes over te sturen via nodes.

`ros::Publisher Driver`
Maak een Publisher aan, deze is global. Dit wordt gebruikt om informatie te versturen via nodes. Als dit wordt aangeroepen wordt de informatie op de bus gezet. Als een andere node luistert naar deze specifieke publisher, dan ontvangt hij de data.
## De code voor functies:
### lowpassFilter
het lowpass filter wordt gebruikt om  willekeurige uitschieters in waardes te voorkomen waneer het object in en uit het beeld beweegt. Het lowpass filter wordt gebruikt voor het de acceleratie/voorwaardse beweging van de robot. 
```C++ 
smoothData = smoothData - (LPF_Beta * (smoothData - rawData));
```
Dit is het belangrijkste deel van het lowpass filter, hierbij is rawData de input van vision(afstand van het object), smooth data is de gefilterde data(uitkomst vorige filtering, dit is t-1) en LPF_Beta bepaald de kracht van de filtering.
In deze berekening wordt het verschil tussen smoothData(t-1) en rawData vermenigvuldigd door LPF_Beta, de oude smooth data min deze uitkomst is de nieuwe smooth data. Dit onderdrukt de pieken in de data wat voor een rustiger rijpatroon zorgt
### drive
```
void drive(double speed, double rotation){
    geometry_msgs::Twist driveObj;
    driveObj.angular.z = rotation;
    driveObj.linear.x = speed;
    ROS_INFO("x: [%f], z: [%f]", rotation, speed);
    driver.publish(driveObj);
} 
```
Dit is de functie drive(). Deze accepteerd de argumenten `double speed` en `double rotation`. Hij gebruikt de standaard message van ros `geometry_msgs::Twist` wat een vector mee geeft. Hij geeft een angular vector en een linear vector mee voor de rotatie en snelheid. Deze wordt even geprint, voor debug toepassingen en wordt daarna gepublished als `driveObj`. Deze waarde wordt dan geadvertised.

### kalmanFilter
De functie kalmanFilter is het duidelijkst als je de onderstaande filmpjes kijkt, deel 2/5 beschrijft de werking van het kalman filter.
[Uitleg kalmanfilter](https://www.youtube.com/watch?v=tk3OJjKTDnQ)
Het kalman filter gebruiken we voor het voorkomen van willekeurige uitschieters in waardes waneer het object in en uit het beeld beweegt. Het kalman filtert in ons geval alleen de draaibewegingen.
De reden voor een kalman filter is dat een kalman filter snelle veranderingen in waardes vertraagd, een korte piek in waardes zal weinig effect hebben, een lang aanhoudende grote waarde wel. 

De input voor de functie is ```measurment``` en ```speed```(deze kan genegeerd worden)
Measurment wordt door het kalman filter gehaald(zie filmpje) en de ```estimate``` wordt in ```rotation``` gezet om daarna door de drive functie naar de motor gestuurd te worden 

Ons idee was om ook de voorspellende werking van het kalman filter te gebruik om zo de robot in dezelfde richting te laten sturen als het object te snel draait en uit het beeld verdwijnt. Dit hebben we niet afgemaakt maar hiervoor zou een aparte node gemaakt moeten worden om het kalman filter constand op output data te laten draaien. Hiervoor staan er nogsteeds de variabelen ```newData``` en ```loopcount``` in kalmanFilter.

### trackedCallback
```
void trackedCallback(const vision::TrackedPositionConstPtr& msg) {
    //speed
    newData = true;
    loopCount = 0;
	ROS_INFO("TRACKED CALLBACK");
    double speed = lowpassFilter(msg->z);
    if (speed > 4)
        speed = 4;
        
	//rotation
    kalmanFilter(-msg->x, speed);
}
```
Dit is de functie trackedCallback(). Deze functie handeld de data af van vision (het zicht van de camera). Deze data komt dus van een Subscriber die in de main staat. Zolang hij nieuwe data krijgt reset de loopcount voor het kalman filter. Hier wordt ook opnieuw iets geprint voor debug toepassingen. De snelheid gaat eerst door een lowpassFilter (msg->z is hier de lineare data, aldus de snelheid...) en daarna gaat hij de gefilterde snelheid gebruiken om te rijden naar het object wat hij ziet. Voor de angulaire data gebruiken we een kalman filter. Dit geeft ook mogelijkheid voor het voorspellen van het pad dat het object aflegd.

### main
```
int main(int argc, char **argv) {
  ROS_INFO("Starting the node");

  ros::init(argc, argv, "angular_tracker");
  ROS_INFO("ros::init done");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/vision/tracked_position", 100, trackedCallback);
  driver = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

  ros::spin();

  ROS_INFO("Exiting the node");
  return 0;
}
```
In de main worden er eerst een paar dingen geprint. Daarna gebruiken we de `ros::init` om het mogelijk te maken om de roscpp functies te gebruiken. We maken een handler aan voor de nodes en daarna de subscriber, die luisterd naar berichten die `vision/tracked_position` advertised. Als hier iets wordt opgepikt, gaat deze naar de callback. Hier handelen we ook de driver advertiser af, indien deze wordt gepublished via de functie void drive(). Dit gebeurd met de standaard message voor vectoren en de node voor het onderstelsel van de robot luisterd hier naar.














#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tinyxml2.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include "std_msgs/Int32.h" // Para la simulacion real
#include "CFuzzySpeedController.h" // Para el control de velocidad

struct Point {
    double x;
    double y;
    double theta;
};
struct Vector {
    double x;
    double y;
};

double quatx = 0;
double quaty = 0;
double quatz = 0;
double quatw = 0;

// Declaracion de variables
std::vector<float> laser_ranges; // Los valores queda el laser son float
std::vector<double> vector_vff = {0.0, 0.0};
std::vector<double> target_vector = {0.0, 0.0};
std::vector<double> repulsion_vector = {0.0, 0.0};
Point arrayOfPoints[4];
Point robotPosicion;
double roll, pitch, yaw;
double error_orientation = 0;
double error_distance = 0;
float linearposx;
float linearposy;
// Control difuso
double linVel;
double angVel;
CFuzzySpeedController controller("Fuzzy Speed Controller", "Speed controller");

void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
	// std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << ", w:"<< msg->pose.pose.orientation.w << "}" << std::endl;
	robotPosicion.x = msg->pose.pose.position.x;
	robotPosicion.y = msg->pose.pose.position.y;
	// point_z = msg->pose.pose.position.z;
	// orien_w = msg->pose.pose.orientation.w;

    linearposx=msg->pose.pose.position.x;
    linearposy=msg->pose.pose.position.y;
    quatx= msg->pose.pose.orientation.x;
    quaty= msg->pose.pose.orientation.y;
    quatz= msg->pose.pose.orientation.z;
    quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, robotPosicion.theta);
    robotPosicion.theta *= 180/M_PI; // Convierte a grados
    // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
    ROS_INFO("RobotPostion: (%f,%f,%f)",robotPosicion.x,robotPosicion.y,robotPosicion.theta); // Funciona bien
}
// Para ver las medidas debe existir el TOPIC del laser, para ello usar la GUI del simulador.
// Asumiendo un eje cartesiano  cuyo angulo cero esta al ESTE y aumenta en sentido antihorario,
// en este robot, el valor cero apunta al valor que esta a la espalda de donde apunta el robot.
// Por lo tanto, la posicion del array con la distancia en el angulo de apuntamiento del robot
// es 180.
void laserCallback(const sensor_msgs::LaserScanPtr& msg){
    
    // ROS_INFO("Longitud del laser de ROS: %ld",msg->ranges.size());
    // Redimensionamos el tamano del vector para evitar problemas.
    laser_ranges.resize(msg->ranges.size());
    // Guarda el array de distancias. El angulo de apuntamiento esta en la posicion 180.
    laser_ranges = msg->ranges;
}

void try_move(Point robotPosic, geometry_msgs::Twist &speed, Point &robotTarget){
    std::cout << "--------------------------------------" << std::endl;
    error_orientation = atan2((robotTarget.y-robotPosic.y),(robotTarget.x-robotPosic.x))*180/M_PI - robotPosic.theta;
    error_distance = sqrt(pow((robotTarget.x-robotPosic.x),2) + pow((robotTarget.y-robotPosic.y),2));
    controller.getSystemInput(error_distance, error_orientation, &linVel,  &angVel);
    speed.linear.x = linVel;
    speed.angular.z = angVel;
}

int cargarXML();

int main(int argc, char** argv){

	ros::init(argc,argv,"thirdProj");	
	ros::NodeHandle nh;	

    cargarXML();
    
    // Para la simulacion
	ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1000);
    ros::Subscriber laser_meas = nh.subscribe("/robot0/laser_0",1000, laserCallback);
    ros::Subscriber odom = nh.subscribe("/robot0/odom",1000, odometryCallback);
	
    // Para el robot real
    // ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    // ros::Subscriber laser_meas = nh.subscribe("/scan",1000, laserCallback);
    // ros::Subscriber odom = nh.subscribe("/pose",1000, odometryCallback);
    // ros::Publisher motor_pub = nh.advertise<std_msgs::Int32>("/cmd_motor_state",1);
    // std_msgs::Int32 enable;
    // enable.data=1;
    // motor_pub.publish(enable);

    // ========================================================
    // Empieza el bucle WHILE
    // ========================================================
    
    ros::Rate loop(10); // Ejecuta a hercios
	while(ros::ok()){ // Espera a que el master este listo para comunicarse
		geometry_msgs::Twist speed;
		//geometry_msgs::Pose position;
        
        // Genera ERROR de SegmentationFault
        try_move(robotPosicion,speed,arrayOfPoints[0]);

        // Necesario para la activacion del motor del robot real.
        // std_msgs::Int32 enable;
        // enable.data=1;
        // motor_pub.publish(enable);

        // Publica la velocidad.
        speed_pub.publish(speed);

		ros::spinOnce();
		loop.sleep();
	}

    return 0;
}

int cargarXML(){
    //---------------------Parte del XML----------------------------
    tinyxml2::XMLDocument doc;
    doc.LoadFile("/home/alumno/robotica_movil_ws/src/thirdProj/src/puntos.xml");
    if (doc.Error()) {
        std::cout << "Error al cargar el XML!" << std::endl;
        return -1;
    }
    // Obtiene el elemento 'nav-points'
    tinyxml2::XMLElement* navPoints = doc.FirstChildElement("map")->FirstChildElement("nav-points");
    if (!navPoints) {
        std::cout << "No se encontraron puntos de navegación!" << std::endl;
        return -1;
    }
    // Itera sobre cada elemento 'point'
    for (tinyxml2::XMLElement* point = navPoints->FirstChildElement("point"); point != NULL; point = point->NextSiblingElement("point")) {
        int i=0;
        int x,y;
        point->QueryIntAttribute("x", &x);
        point->QueryIntAttribute("y", &y);
        arrayOfPoints[i].x=x;
        arrayOfPoints[i].y=y;
        std::cout << "Punto: X=" << x << ", Y=" << y << std::endl; 
    }
//-------------------------Fin del XML-------------------------------------------
}

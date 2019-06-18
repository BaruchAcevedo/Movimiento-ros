#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

const int SENSORES = 6;

const int WIDTHC = 11;
const int HEIGHTC = 21;

const float PI = 3.1416;

const float resolution = 1.0;

int x_global=(WIDTHC/2);
int y_global=(HEIGHTC/2);

float direcciones[SENSORES]; //Donde se encuentran, para cada uno de los 8 vectores, los angulos de los vectores, de los cuales se 
                            // calculan las rectas dadá las coordenadas del robot y estos angulos.

double magnitudes[SENSORES];

float direccion[2]; // vector que determina a donde se dirige. Dadas las coordenadas globales.

float objetivo[2];
float angle_objetivo;

float VELOCIDAD_MAXIMA = 0.2;

float x_odom=0.0;
float y_odom=0.0;
float angle_odom=PI;
float vx_odom = 0.0;
float vz_odom = 0.0;

class sensores{
  private:
    double magnitudes[SENSORES];
  public:
    sensores(double m[]);
};

sensores::sensores(double m[]){
  for (int i = 0; i < SENSORES; i++)
  {
    magnitudes[i]=m[i];
  }
  
}

class flecha
{
private:
  visualization_msgs::Marker vector;
public:
  flecha(float x_1, float y_1, float x_2, float y_2,int id,double r, double g, double b);
  void publicar(ros::Publisher marker_pub);
};

/**
 * Constructor para la clase flecha, aquí se determinan todas las propiedades de la flecha.
 * @param x_1 Punto donde iniciará la flecha en el eje x.
 * @param y_1 Punto donde iniciará la flecha en el eje y.
 * @param x_2 Punto donde apunta la flecha en el eje x.
 * @param y_2 Punto donde apunta la flecha en el eje y.
 * @param id Identificador del marcador, debé ser único.
 * @param r Componente rojo del color de la flecha.
 * @param g Componente verde del color de la flecha.
 * @param b Componente azul del color de la flecha.
 */
flecha::flecha(float x_1, float y_1, float x_2, float y_2,int id,double r, double g, double b)
{
  vector.header.frame_id = "/odom";
  vector.header.stamp = ros::Time::now();

  vector.id = id;
  vector.ns = std::to_string(id);
  vector.type = visualization_msgs::Marker::ARROW;
  vector.scale.x=0.1;
  vector.scale.y=0.1;
  vector.color.b = b;
  vector.color.r = r;
  vector.color.g = g;
  vector.color.a = 0.5;
  
  geometry_msgs::Point p;
  p.y=y_2;
  p.x=x_2;
  geometry_msgs::Point q;
  q.y=y_1;
  q.x=x_1;
  vector.points.clear();
  vector.points.push_back(q);
  vector.points.push_back(p);

}

/**
 * Función que permite publicar la flecha.
 * @param marker_pub Publicador en el cual se mostrará la flecha.
 */
void flecha::publicar(ros::Publisher marker_pub){
  marker_pub.publish(vector);
}

/**
 * Función para el subscriptor del topico "/distance". Actualiza la variable 
 * global magnitudes respecto a los datos de los sensores.
 * @param f Mensaje en el cual se mostrarán las distancias, debe ser del tamaño de
 * SENSORES.
 */
void actualiza_magnitudes(std_msgs::Float32MultiArray f){
  for(int i = 0; i<SENSORES; i++){
    magnitudes[i]=f.data[i]*0.01;
  }
}

/**
 * Función asociada al subscriptor de "/odom". Actualiza los datos de la posición actual del robot.
 * @param odom Mensaje recivido del odometro.
 */
void actualiza(nav_msgs::Odometry odom){
  x_odom = odom.pose.pose.position.x;
  y_odom = odom.pose.pose.position.y;
  tf2::Quaternion q = tf2::Quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w);
  angle_odom = q.getAngle();
  vx_odom = odom.twist.twist.linear.x;
  vz_odom = odom.twist.twist.angular.z;
  y_global=(int)y_odom+(HEIGHTC/2);
  x_global=(int)x_odom+(WIDTHC/2);

  float avance = (angle_odom-PI/2) - direcciones[0];

  for (int i = 0; i < SENSORES; i++)
  {
    direcciones[i]+=avance;
    if(direcciones[i]>2*PI){
      direcciones[i]-=2*PI;
    }
    if(direcciones[i]<0){
      direcciones[i]+=2*PI;
    }
  }

}

/**
 * Función que permite publicar una nueva velocidad, tanto angular como lineal, y
 * verifica que no se encuentre fuera de los rangos permitidos.
 * @param vx Nueva velocidad lineal.
 * @param vth Nueva velocidad angular.
 * @param velocity_pub Publicador con el cual se mostrará la velocidad.
 */
void publicar(float vx, float vth, ros::Publisher velocity_pub){
    if(vx > VELOCIDAD_MAXIMA)
      vx = VELOCIDAD_MAXIMA;
    if(vx < -VELOCIDAD_MAXIMA)
      vx = -VELOCIDAD_MAXIMA;
    if(vth > VELOCIDAD_MAXIMA)
      vth = VELOCIDAD_MAXIMA;
    if(vth < -VELOCIDAD_MAXIMA)
      vth = -VELOCIDAD_MAXIMA;

    geometry_msgs::Twist velocidad;

    velocidad.angular.z=vth;
    velocidad.linear.x=vx;

	  velocidad.linear.y = 0;
	  velocidad.linear.z = 0;
	  velocidad.angular.x = 0;
	  velocidad.angular.y = 0;

    velocity_pub.publish(velocidad);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); //Publicador para Rviz.
  ros::Subscriber sub_sensores = n.subscribe("/distance",5,actualiza_magnitudes); //Subscriptor de las distancias medidas.
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); //Publicador para la velicidad del robot.
  ros::Publisher reboot_pub = n.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1); // Publicador para reiniciar los datos del odometro.
  ros::Subscriber sub_odom = n.subscribe("/odom", 1, actualiza); //Subscriptor del odometro.

  reboot_pub.publish(std_msgs::Empty());
  for(int i = 0; i < SENSORES; i++)
  {
    direcciones[i]=((2.0*PI)/(float)SENSORES)*i;
    if(direcciones[i]+angle_odom>2*PI){
      direcciones[i]+=angle_odom-2*PI;
    }else if(direcciones[i]+angle_odom<0){
      direcciones[i]+=angle_odom+2*PI;
    }else
      direcciones[i]+=angle_odom;
  }

  for(int i = 0; i < SENSORES; i++)
  {
    magnitudes[i]=0.0;
  }

  direccion[0]=0.0;
  direccion[1]=0.0;

  objetivo[0]=1.0;
  objetivo[1]=10.0;
  angle_objetivo = 0.0;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1);
  reboot_pub.publish(std_msgs::Empty());
  ROS_INFO("Inicializado");

  while(n.ok()){
    ros::spinOnce();  

    current_time = ros::Time::now();

    if(x_odom != objetivo[1] && y_odom!=objetivo[0]){

      double dt = (current_time - last_time).toSec();
      double vf_x;
      double vf_y;
      double delta_th;

      direccion[0]=0.0;
      direccion[1]=0.0;

      for(int i = 0; i < SENSORES; i++)
      {
        if(magnitudes[i]>0.20){
          direccion[0]=direccion[0]-(1/(magnitudes[i]-0.20))*sin(direcciones[i]);
          direccion[1]=direccion[1]-(1/(magnitudes[i]-0.20))*cos(direcciones[i]);
          flecha(x_odom,y_odom,(magnitudes[i]*cos(direcciones[i]))+x_odom,(magnitudes[i]*sin(direcciones[i]))+y_odom,60+i,1.0,0.5,0.8).publicar(marker_pub);
        }
      }

      float angulo = angle_odom;

  //    ROS_INFO("Vector aceleracion Actual: (%f,%f)",direccion[0],direccion[1]);

      //flecha(x_odom,y_odom,direccion[1]+x_odom,direccion[0]+y_odom,54,1.0,0.7,0.3).publicar(marker_pub);
      //flecha(x_odom,y_odom,objetivo[1],objetivo[0],52,0.1,0.3,0.5).publicar(marker_pub);
      flecha(x_odom,y_odom,direccion[1]+x_odom,direccion[0]+y_odom,57,0.0,1.0,1.0).publicar(marker_pub);
      flecha(x_odom,y_odom,objetivo[1],objetivo[0],50,0.0,1.0,1.0).publicar(marker_pub);

      direccion[0]=(direccion[0])+((objetivo[0])-y_odom);
      direccion[1]=(direccion[1])+((objetivo[1])-x_odom);

      flecha(x_odom,y_odom,direccion[1]+x_odom,direccion[0]+y_odom,52,0.0,1.0,1.0).publicar(marker_pub);
      //                 x                             y   
      double ax = cos(angle_odom)*direccion[1]+sin(angle_odom)*direccion[0];
      double ay =-sin(angle_odom)*direccion[1]+cos(angle_odom)*direccion[0];

      //flecha(x_odom,y_odom,ax+x_odom,ay+y_odom,50,1.0,1.0,1.0).publicar(marker_pub);
      flecha(x_odom,y_odom,ax*cos(angle_odom)+x_odom,ax*sin(angle_odom)+y_odom,53,0.0,1.0,0.0).publicar(marker_pub);
      flecha(x_odom,y_odom,ay*(-sin(angle_odom))+x_odom,ay*cos(angle_odom)+y_odom,54,1.0,0.0,0.0).publicar(marker_pub);
      flecha(x_odom,y_odom,cos(angle_odom)+x_odom,sin(angle_odom)+y_odom,55,1.0,1.0,1.0).publicar(marker_pub);
      flecha(x_odom,y_odom,(-sin(angle_odom))+x_odom,cos(angle_odom)+y_odom,56,1.0,1.0,1.0).publicar(marker_pub);
      //ROS_INFO("tIEMPO %f",dt);

      double angular=vz_odom+ay*dt;
      double lineal=vx_odom+ax*dt;
      if((angular == 0 && lineal==0) ){
        ax = cos(angle_odom)*objetivo[1]+sin(angle_odom)*objetivo[0];
        ay =-sin(angle_odom)*objetivo[1]+cos(angle_odom)*objetivo[0];
        angular=vz_odom+ay*dt;
        lineal=vx_odom+ax*dt;
      }

      flecha(x_odom,y_odom,lineal+x_odom,angular+y_odom,51,0.0,0.0,1.0).publicar(marker_pub);

      //ROS_INFO("%f %f",angular,lineal);
      double aumento = 30.0;
      if(vx_odom<=0.01 && vz_odom <= 0.01){
        publicar(lineal*aumento,angular*aumento,velocity_pub);
      }else{
        publicar(lineal,angular,velocity_pub);
      }

      publicar(lineal,angular,velocity_pub);

    }else if(angle_odom!=angle_objetivo){

      if(vx_odom!=0){
        publicar(0,0,velocity_pub);
      }
      publicar(0,0.1,velocity_pub);
    }else{
      ROS_INFO("Objetivo alcanzado");
      ros::shutdown();
    }

    last_time = current_time;
  }
}
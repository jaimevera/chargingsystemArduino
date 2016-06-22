

//Comandos en ROS
//1- roscore
//2- rosrun rosserial_python serial_node.py /dev/ttyUSB0
//3- rostopic echo batteryVolt
//4- rostopic echo encoder_1
//5- rostopic pub your_topic std_msgs/Int32 4

//5- rostopic pub -r 1 your_topic std_msgs/Int32 40      -Repeticion cada 1 Hz

/*Arduino code for MD23 and RFR08 control, using the wire and ROSserial librarys*/

#include <Wire.h>                  //Libreria necesaria para la comunicacion mediante el protocolo I2C
#include <ros.h>                   //Libreria para la comunicacion con ROS
#include <std_msgs/Int32.h>        //Libreria para el tipo de mensaje intercambiado con ROS

ros::NodeHandle nh;                //Inicializa y establece las conexiones con los nodos correspondientes
std_msgs::Int32 enc_1;
ros::Publisher pe1("encoder1", &enc_1);  //Declaracion de los topicos en los que se publica desde Arduino. Nombre del topic en ROS: "encoder1"
std_msgs::Int32 enc_2;
ros::Publisher pe2("encoder2", &enc_2);  
std_msgs::Int32 batt;
ros::Publisher pbv("batteryVolt", &batt);  

std_msgs::Int32 RF_L;
ros::Publisher pL("sensorRFL", &RF_L); 
std_msgs::Int32 RF_C;
ros::Publisher pC("sensorRFC", &RF_C);  
std_msgs::Int32 RF_R;
ros::Publisher pR("sensorRFR", &RF_R);  



int x = 0;                    
int y = 0;
int reading=0;

void messageCb1(const std_msgs::Int32& msg1){        //Establecen el tipo de mensajes que se reciben de ROS para los topicos de velocidades
    x=msg1.data;
}

void messageCb2(const std_msgs::Int32& msg2){
    y=msg2.data;
}

ros::Subscriber<std_msgs::Int32> sub1("speed1", &messageCb1);    //Establecen los nombres de los topicos a los que se subscribe Arduino y el tipo de mensaje anteriormente definido

ros::Subscriber<std_msgs::Int32> sub2("speed2", &messageCb2);



#define md23Address 89               // Direccion de la tarjeta MD23

                                     //Correspondencia entre los bits de los registros y el nombre de cada variable   
#define softwareReg 13   //0x0D      // Byte to read the software version
#define speed1 0  //0x00             // Byte to send speed to first motor
#define speed2 1  //0x01             // Byte to send speed to second motor
#define cmdByte 16   //0x10          // Command byte
#define encoderOne 2    //0x02       // Byte to read motor encoder 1
#define encoderTwo 6    //0x06       // Byte to read motor encoder 2
#define voltRead 10      //0x0A      // Byte to read battery volts


#define modeOp 15                          //Modo operacion.


void setup(){                  //Funcion inicial, se ejecuta al principio

  
  nh.initNode();               //Inicializa los nodos y la publicacion de los diferentes topicos.
  nh.advertise(pe1);
  nh.advertise(pe2);
  nh.advertise(pbv);
  
  nh.advertise(pL);
  nh.advertise(pC);
  nh.advertise(pR);
  
  nh.subscribe(sub1);          //Inicializa la subscripcion a los mensajes con las velocidades para los motores.
  nh.subscribe(sub2);
  
  
  Wire.begin();                //Inicializa las comunicaciones I2C.


  Wire.beginTransmission(md23Address);                    // Establece el modo de funcionamiento de la MD23 en modo 3.
  Wire.write(modeOp);
  Wire.write(3);                                           
  Wire.endTransmission();

  encodeReset();    // Llamada a la funcion que resetea los valores de los encoders.
  
  
}



void loop(){          //Bucle principal

 
  RF_C.data =lectura (127);     //Lectura sensor RFR08 (Central, con direccion 127)            
  pC.publish( &RF_C );
  nh.spinOnce();
  
  RF_L.data = lectura (113);    //Lectura sensor RFR08 (Izquierdo, con direccion 113)            
  pL.publish( &RF_L );
  nh.spinOnce();
  
  RF_R.data  = lectura (114);    //Lectura sensor RFR08 (Derecho, con direccion 114)           
  pR.publish( &RF_R );
  nh.spinOnce();
 
                                                        
    Wire.beginTransmission(md23Address);                    // Avance del robot con la velocidad x
    Wire.write(speed1);
    Wire.write(x);                                           
    Wire.endTransmission();
  
    Wire.beginTransmission(md23Address);                    // Giro del robot con la velocidad y
    Wire.write(speed2);
    Wire.write(y);
    Wire.endTransmission();

    
    encoder1();              //Llamada a las funciones que realizan las lecturas en los registros de encoders y tension, y que realizan que publican sus valores en el topico correspondiente
    encoder2();
    volts();
    
 
}

int getSoft(){                                              // Funcion para obtener la version del software
  Wire.beginTransmission(md23Address);                      
  Wire.write(softwareReg);
  Wire.endTransmission();
  Wire.requestFrom(md23Address, 1);                        
  while(Wire.available() < 1);                              
  int software = Wire.read();                            
  
  return(software);
}

void encodeReset(){                                         // Funcion para resetear los encoders
  Wire.beginTransmission(md23Address);
  Wire.write(cmdByte);
  Wire.write(32);
  Wire.endTransmission(); 
}

void encoder1(){                                            // Funcion para leer el registro con el valor del encoder 1
  Wire.beginTransmission(md23Address);                      
  Wire.write(encoderOne);
  Wire.endTransmission();
  
  Wire.requestFrom(md23Address, 4);                        
  while(Wire.available() < 4);                              
  long firstByte = Wire.read();                          
  long secondByte = Wire.read();
  long thirdByte = Wire.read();
  long fourthByte = Wire.read();
  
  long poss1 = (firstByte << 24) + (secondByte << 16) + (thirdByte << 8) + fourthByte;  
  
                                          
  enc_1.data =poss1;                
  pe1.publish( &enc_1 );                                    //Publicacion en el topico encoder1 con el valor del encoder1
  nh.spinOnce();

}

void encoder2(){                                            // Funcion igual a la anterior para el encoder2
  Wire.beginTransmission(md23Address);                                      
  Wire.write(encoderTwo);
  Wire.endTransmission();
  
  Wire.requestFrom(md23Address, 4);                         
  while(Wire.available() < 4);                              
  long firstByte = Wire.read();                         
  long secondByte = Wire.read();
  long thirdByte = Wire.read();
  long fourthByte = Wire.read();
  
  long poss2 = (firstByte << 24) + (secondByte << 16) + (thirdByte << 8) + fourthByte;  

  enc_2.data =poss2;                
  pe2.publish( &enc_2 );
  nh.spinOnce();

}

void volts(){                                               // Funcion para realizar la lectura de la tension
  Wire.beginTransmission(md23Address);                      
  Wire.write(voltRead);
  Wire.endTransmission();
  
  Wire.requestFrom(md23Address, 1);                         
  while(Wire.available() < 1);                              
  int batteryVolts = Wire.read();                       

  batt.data =batteryVolts;                                  //Publicacion en el topico batteryVolts con el valor de la tension  
  pbv.publish( &batt );
  nh.spinOnce();

}

void stopMotor(){                                           // Funcion para parar los motores
  Wire.beginTransmission(md23Address);
  Wire.write(speed2);
  Wire.write(0);                                          
  Wire.endTransmission();
  
  Wire.beginTransmission(md23Address);
  Wire.write(speed1);
  Wire.write(0);                                           
  Wire.endTransmission();
}  




int lectura (int address) {                                  //Funcion para las lecturas de los sensores ultrasonicos RFR08
  
  int reading;
  
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(address); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
  // use 0x51 for centimeters
  // use 0x52 for ping microseconds
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  delay(70);                   // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(address); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(address, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }

  return reading;
}

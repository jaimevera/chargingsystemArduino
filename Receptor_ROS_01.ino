
/* Arduino Code for the reception control of IR signals and publication of messages in the ROS topic*/


// Mapeo de los pines desde los puertos http://arduino.cc/en/Hacking/PinMapping168 ('raw' pin mapping)
// Puertos, pines, mascaras, etc: https://www.arduino.cc/en/Reference/PortManipulation
// Explicacion bucles http://forum.arduino.cc/index.php?topic=110299.0

 
#include <ros.h>                   //Libreria ROSserial para comunicacion con ROS
#include <std_msgs/Int32.h>        //El mensaje para la comunicacion serie con ROS es un Int32

ros::NodeHandle nh;                
std_msgs::Int32 test;
ros::Publisher p("codigoIR", &test);  //Nombre del topic en ROS: "codigoIR"


#define IRpin_PIN PIND  //Puerto D, que incluye los pines digitales 2, 3, 4 y 5 del Arduino UNO. (B00111100) 
                        //Receptor IR Derecho           -> pin 2  (B00000100)    (Visto desde el robot/arriba)
                        //Receptor IR Central Derecho   -> pin 3  (B00001000)
                        //Receptor IR Central Izquierdo -> pin 4  (B00010000)
                        //Receptor IR Izquierdo         -> pin 5  (B00100000)


#define MAXPULSE 8000   //Tiempo para detectar las pausas entre mensajes consecutivos
#define RESOLUTION 20


uint8_t currentpulse = 0;  //Indice para el pulso que se esta guardando
uint8_t num_pulsos = 0;    //Numero de pulsos para cada ciclo

void setup(void) {

  nh.initNode();
  nh.advertise(p);


  DDRB = DDRB | B00001110;  // Se establecen los puertos 1, 2 y 3 como salidas para encender el led RGB.
                            //Pin 9  para led rojo  -> PORTB = B00000010;
                            //Pin 10 para led verde -> PORTB = B00000100;
                            //Pin 11 para led azul  -> PORTB = B00001000;
}

void loop(void) {

  int contador_inicial, leer_codigo;      //Contador para el tiempo del flanco positivo de inicio de codigo
  contador_inicial = leer_codigo = 0;     //Leer_codigo->variable igual a 1 si se detecta el inicio de codigo para leerlo a continuacion
  int contador2=0;


  int codigo0 = B00000000;          //Variables para guardar las lecturas de los receptores
  long codigo=0;
  long codigo1=0;
  long codigo2=0;
  long codigo3=0;
  
  long codigo1b=0;
  long codigo2b=0;
  long codigo3b=0;  
  
  long codigo4=0;


  //Bucle para detectar el pulso inicial en el que los 3 emisores estan emitiendo a la vez.
  //Cuando un receptor IR recibe señal, su pin correspondiente esta en LOW, cuando recibe señal en HIGH
  
  while ((IRpin_PIN & B00111100)==60) {    //Si ninguno de los receptores IR recibe señal, sus pines estan en HIGH (IRpin_PIN=B00111100) 
                                            //Luego la operacion AND es igual a B00111100 (60 en binario)
    contador_inicial++;
    delayMicroseconds(20);        
    
    if ((20*contador_inicial>= 5000*MAXPULSE) && (leer_codigo==0)){    //Para publicar un primer mensaje (=0) en el caso de no recibir nada, ya que en ese caso, no entra en el bucle y no publica nada
        test.data = 4096;                                              //Se publica el mensaje igual a 0 cada aprox 5000 posibles lecturas de codigo
        p.publish( &test );
        nh.spinOnce();
        
        delay(10);
        return;
      }
  }
     
    //Si el tiempo durante el que no reciben señal es mayor que MAXPULSE y ademas se recibe la señal de algun led IR, entonces leer_codigo=1

    if ((20*contador_inicial>= MAXPULSE) && ((IRpin_PIN & B00111100)!=60)){  
      while ((IRpin_PIN & B00111100)!=60){
         contador2++;
         delayMicroseconds (20);
         
         if ((20*contador2) >=1800) {    //Mide el pulso IR (de todos los emisores a la vez) que dura 2000 microsegundos
      
            leer_codigo = 1;            //Variable para pasar al siguiente estado
            num_pulsos = 0;
         }
      }  
    }

    
    

    if (leer_codigo == 1) {       //Mientras leer_codigo=1, se comprueban las señales en tres tiempos distintos para detectar que señales recibe cada receptor IR
                                  //En cada pulso se leen las entradas de cada receptor IR (3 bits) y se guardan para formar un numero binario de 9 bits formado 
                                  //por la concatenacion de las 3 lecturas de los 3 receptores de 3 bits cada una.

      //METODO: GUARDA LOS VALORES DE LA SEÑAL EN DISTINTOS TIEMPOS, QUE CORRESPONDEN A LOS PULSOS DE CADA LED IR

      delayMicroseconds (3000);          //El flanco emitido por el led IR Derecho se produce desde 4500 a los 5500 microsegundos desde el inicio
      codigo1b=~IRpin_PIN & B00111100;   //Operacion AND que da como resultado un 1 (si recibe señal) o un 0 (si no recibe) para cada uno de los receptores. 
                                         //Si IRpin_PIN=B00010100, ~IRpin_PIN=B11101011, el resultado de la operacion es: B00001000  (recibiendo el receptor central)
                                        

      delayMicroseconds (2500);          //El flanco emitido por el led IR Central se produce 2500 microsegundos desde el inicio del pulso anterior
      codigo2b=~IRpin_PIN & B00111100;



      delayMicroseconds (2500);          //El primer emitido por el led IR Izquierdo se produce 2500 microsegundos desde el inicio del pulso anterior
      codigo3b=~IRpin_PIN & B00111100;


      codigo1=codigo1b<<6;              //Se desplaza la posicion de los bits para unirlos en una unica variable binaria
      codigo2=codigo2b<<2;
      codigo3=codigo3b>>2;
      
      
      codigo1b = codigo1 | 4096L;       //4096 en binario: 1000000000000   = un UNO y 12 CEROS
      codigo2b = codigo2 | 4096L;
      codigo3b = codigo3 | 4096L;


      codigo0 = codigo1b | codigo2b;    //Suma de los codigos leidos anteriormente. Operacion OR que devuelve un 1 si en cualquiera de los operandos hay un 1.
      codigo = codigo0 | codigo3b;      //El codigo final es la suma de los 3 codigos leidos en los distintos pulsos.
  

      test.data = codigo;               //Publicacion a ROS del codigo formado por los bits de todas las lecturas de cada receptor
      p.publish( &test );
      nh.spinOnce();
      
      
      leer_codigo=0;
      
      delay(10);


      //CODIGO ENCENDIDO LED RGB (para un solo receptor y usado para realizar comprobaciones del funcionamiento y ajustar frecuencias) 
      
//      if ((codigo & 10) != 0) {                              //Si recibe señal del led derecho, se enciende el led azul
//        PORTB = PORTB | B00001000;      //Color Azul
//      }
//      if ((codigo & 100000) != 0) {                          //Si recibe señal del led central, se enciende el led verde
//        PORTB = PORTB | B00000100;       //Color Verde
//      } 
//      if ((codigo & 1000000000) != 0) {                       //Si recibe señal del led izquierdo, se enciende el led rojo
//        PORTB = PORTB | B00000010;      //Color Rojo
//      }
//                                                               //En el caso de que reciba 2 o las 3 señales, se encienden a la vez dando lugar a los colores secundarios
//      delay(10);              //Tiempo de espera para mostrar led
//      PORTB = B00000000;      //Reinicia los valores del led RGB a 0
//      
      
      return;

    
  }
}




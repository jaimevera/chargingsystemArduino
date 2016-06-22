/*
http://www.davidmiguel.com/arduino/dia-3-sensor-ir/

*/

/* Arduino Code for sending IR messages with 3 IR emitters */

/*
PUERTOS
  Se usan directamente las direcciones de los pines correspondientes,
  para evitar usar la funcion digital write (que tarda 3microsegundos en ejecutarse)
    
    Digital pin 2 --> PD2 -- Izquierdo
    Digital pin 3 --> PD3 -- Central
    Digital pin 4 --> PD4 -- Derecho
*/


byte portD_Izquierdo = B00000100;      //Declaracion de los puertos para los pines de cada emisor
byte portD_Central =   B00001000;
byte portD_Derecho =   B00010000;
byte portD_Todos   =   B00011100;      //Declaracion de todos los pines a la vez
byte portD_Ninguno =   B00000000;

 
void setup() {
  
  DDRD = DDRD | B11111100;  // Se establecen los puertos 2-7 como salidas
                            // Sin cambiar los valores del puerto 0 y 1 (RX y TX)

}

void loop(){
  
  //Se ejecuta un ciclo con el codigo diseñado
  //total ciclo = 2000+2500+3*1000+2*1500 = 10500 microsegundos (sin la pausa final)
  
  pulseIR(2000);                //Un pulso inicial con todos los leds IR emitiendo
  delayMicroseconds(2500);      //Pausa
  pulseIRD(1000);               //Emite el led Derecho
  delayMicroseconds(1500);      //Pausa
  pulseIRC(1000);               //Emite el led Central
  delayMicroseconds(1500);      //Pausa
  pulseIRI(1000);               //Emite el led Izquierdo

  delay(20);                    //Pausa final

}

 
void pulseIR(long microseg){    //Funcion para modular la señal mientras se produce un pulso en alto para todos los emisores
  
  //Genera los pulsos de 38KHz de frecuencia durante x microsegundos.

  /*38KHz son aprox. 13microseg en HIGH y 13microseg en LOW
    la instrucción digitalWrite tarde 3microseg en ejecutarse
    por lo que hacemos dos delays de 10 en vez de 13.
    En total el ciclo dura 26microseg, cuando se completa,
    restamos al tiempo que tiene que estar mandando el pulso*/
    
  cli(); //Desabilita cualquier interrupción
  while (microseg > 0) {
    
    PORTD = portD_Todos;            //Pulso en alto (valor 1) de todos los emisores
    delayMicroseconds (13);    
    PORTD = portD_Ninguno;          //Pausa (valor 0) de todos los emisores
    delayMicroseconds (13);    
    microseg -= 26;
  }
sei(); //Activa las interrupciones
}

 
void pulseIRD(long microseg){        //Igual funcion anterior, pero solo para el emisor derecho
 
  cli(); 
  while (microseg > 0) {
    PORTD = portD_Derecho;
    delayMicroseconds (13);
    PORTD = portD_Ninguno;
    delayMicroseconds (13); 
    microseg -= 26;
  }
  sei(); 
}

 
void pulseIRI(long microseg){         //Igual funcion anterior, pero solo para el emisor izquierdo

  cli(); 
  while (microseg > 0) {
    PORTD = portD_Izquierdo;
    delayMicroseconds (13);
    PORTD = portD_Ninguno;
    delayMicroseconds (13); 
    microseg -= 26;
  }
  sei(); 
}

void pulseIRC(long microseg){         //Igual funcion anterior, pero solo para el emisor central
 
  cli();
  while (microseg > 0) {
    PORTD = portD_Central;
    delayMicroseconds (13);
    PORTD = portD_Ninguno;
    delayMicroseconds (13); 
    microseg -= 26;
  }
  sei();
}

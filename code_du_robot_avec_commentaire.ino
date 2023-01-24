

#include <Servo.h>                                                                //Servo motor library. This is standard library
#include <NewPing.h>                                                              //Ultrasonic sensor function library. You must install this library
//les pin reservés au shield L298
const int M1_Direction = 12; //  précise la direction du moteur 1 ( avance si HIGH)        
const int M1_Frein = 9; // freine mot 1 si HIGH
const int M2_Direction = 13; //  précise la direction du moteur 2 ( avance si HIGH)        
const int M2_Frein = 8; // freine mot 2 si HIGH
const int buzzer = 2; 

int pin_d = 7;   // Senseur DOUT (digitale)
int pin_a = A1;  // Senseur AOUT (analogique)

int niveau_senseur = 300;                                                                              

                                                            //les pins du capteur ultrasons
#define trig_pin A4                                        //le pin trig(sortie) du capteur ultrasons est attaché à A4
#define echo_pin A5                                        //le pin echo (entrée) du capteur ultrasons est attaché à A5
#define distance_maximal 200                               // définir un constante de distance maximal dont le capteur le prend en compte*
boolean goesForward = false;                               // l'état du robot
int distance = 100;

NewPing sonar (trig_pin, echo_pin, distance_maximal);              // initialisation de la fonction newPing par (A4,A5,200cm distance maximale*)
Servo servo_moteur;                                               //nommer le servo moteur


//////////////////////////////////////////////////////////////////////////////////VOID SETUP////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){ 
                                                                 //configuration des pin dédiés au shield L298 comme sorties
  pinMode(M1_Direction, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(M1_Frein, OUTPUT);
  pinMode(M2_Direction, OUTPUT);
  pinMode(M2_Frein, OUTPUT);
    pinMode(pin_d, INPUT);
  pinMode(pin_a, INPUT);

  // Definir le buzzer et LEDs comme sortie

  pinMode(buzzer, OUTPUT);

  servo_moteur.attach(10);                                        //attacher le servomoteur sur la pin 10
  digitalWrite(A3, HIGH);
  servo_moteur.write(90);                                         //deplacer le servo moteur 45° de 90° vers 115°
  delay(2000);                                                    //attendre 2 sec
  distance = readPing();                                          //lire le distance en cm
  delay(100);                                                     //attendre 0.1 sec
  distance = readPing();                                          //lire le distance en cm
  delay(100);                                                     //attendre 0.1 sec
  distance = readPing();                                          //lire le distance en cm
  delay(100);                                                     //attendre 0.1 sec
  distance = readPing();                                          //lire le distance en cm
  delay(100);                                                     //attendre 0.1 sec
}

//////////////////////////////////////////////////////////////////////////////////////VOID LOOP/////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);
  // Lecture de DOUT du senseur sur l'entree digital
 int valeur_digital = digitalRead(pin_d);
  // Lecture de AOUT du senseur sur l'entree analogique
  int valeur_analogique = analogRead(pin_a); 
   if (valeur_analogique > niveau_senseur)
  {
  
    
digitalWrite(buzzer, HIGH);
delay(500);
digitalWrite(buzzer, LOW);

  }
else {
  if ((distance <= 40)&&(distance >= 7 )) {                                    //si il y a un obstacle
    moveStop();                                                                //alors freine le robot
    delay(100);                                                                //attendre 0.3 sec
    moveBackward();                                                            // et récule le robot
    delay(500);                                                                //attendre 0.4 sec
    moveStop();                                                                // freine le robot
    delay(300);                                                                //attendre 0.3 sec
    distanceRight = lookRight();                                               // voir à droite et lire la distance
    delay(300);                                                                // attendre 0.3 sec
    distanceLeft = lookLeft();                                                 // voir à gauche et lire la distance
    delay(300);                                                                //attendre 0.3 sec

        if (distance >= distanceLeft){                                         //si il y a un obstacle aussi à gauche
          turnLeft();                                                         //alors tourne à gauche
          moveStop(); 
              moveForward();
              delay(200);// s'il n y a pas un obstacle alors marche avant
//freine le moteur
        }
        else{
          turnRight();                                                          //sinon tourne à droite
          moveStop(); 
              moveForward();
              delay(200);// s'il n y a pas un obstacle alors marche avant
// freine le moteur
        }
  }
  else
  {
    moveForward();                                                            // s'il n y a pas un obstacle alors marche avant
  }
    distance = readPing(); 
    // tout en lisant la distance
}}
////////////////////////////////////////////////////////////////////////////////////LA FONCTION LOOKRIGHT///////////////////////////////////////////////////////////////////////////////////////////////////////////

int lookRight(){                                                               //cette fonction retourne la distance à droite du robot
  servo_moteur.write(10);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_moteur.write(90);
  return distance; 
}
////////////////////////////////////////////////////////////////////////////////////LA FONCTION LOOKLEFT//////////////////////////////////////////////////////////////////////////////////////////
int lookLeft(){                                                                //cette fonction retourne la distance à gauche du robot
  servo_moteur.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_moteur.write(90);
  return distance;
  delay(100);
}
//////////////////////////////////////////////////////////////////////////////////LA FONCTION READPING/////////////////////////////////////////////////////////////////////////////////////////////////
int readPing(){                                                               //cette fonction retourne la distance en cm
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}
/////////////////////////////////////////////////////////////////////////////LA FONCTION MOVESTOP//////////////////////////////////////////////////////////////////////////////////////////////
void moveStop(){                                                                 // cette fonction permet le freinage du robot
  
  digitalWrite(M1_Frein, LOW);
  digitalWrite(M2_Frein, LOW);
  digitalWrite(M1_Direction, LOW);
    digitalWrite(M2_Direction, LOW);
  
}
///////////////////////////////////////////////////////////////////////////LA FONCTION MOVEFORWARD////////////////////////////////////////////////////////////////////////////////////////////
void moveForward(){                                                              //cette fonction faire tourner les deux moteurs dans la mème sens en avant

  if(!goesForward){                                                               //condition si le moteur est en arret alors...

    goesForward=true;
    digitalWrite(M1_Direction, HIGH);
    digitalWrite(M1_Frein, LOW);
    digitalWrite(M2_Direction, HIGH);
    digitalWrite(M2_Frein, LOW);
   
  }
}
//////////////////////////////////////////////////////////////////////////LA FONCTION MOVEBACKWARD/////////////////////////////////////////////////////////////////////////////////////////////
void moveBackward(){                                                             //cette fonction faire tourner les deux moteurs dans la mème sens en arière

    goesForward=false;
    digitalWrite(M1_Direction, LOW);
    digitalWrite(M1_Frein, HIGH);
    digitalWrite(M2_Direction, LOW);
    digitalWrite(M2_Frein, HIGH);

  
}
///////////////////////////////////////////////////////////////////////////LA FONCTION TURNRIGHT////////////////////////////////////////////////////////////////////////////////////////////////////
void turnRight(){                                                               // cette fonction permet au moteur de tourner à droite

 
    digitalWrite(M1_Direction, HIGH);
    digitalWrite(M1_Frein, LOW);
    digitalWrite(M2_Direction, LOW);
    digitalWrite(M2_Frein, LOW);
    delay(800);
    digitalWrite(M1_Direction, HIGH);
    digitalWrite(M1_Frein, LOW);
    digitalWrite(M2_Direction, HIGH);
    digitalWrite(M2_Frein, LOW);
 
 
  
  
}
//////////////////////////////////////////////////////////////////////////////LA FONCTION TURN LEFT////////////////////////////////////////////////////////////////////////////////////////////
void turnLeft(){                                                        // cette fonction permet au moteur de tourner à gauche


    digitalWrite(M1_Direction, LOW);
    digitalWrite(M1_Frein, LOW);
    digitalWrite(M2_Direction, HIGH);
    digitalWrite(M2_Frein, LOW);
    delay(800);
    digitalWrite(M1_Direction, HIGH);
    digitalWrite(M1_Frein, LOW);
    digitalWrite(M2_Direction, HIGH);
    digitalWrite(M2_Frein, LOW);
  
}

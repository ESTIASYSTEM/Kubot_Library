#include "Kubot.hpp"


void Kubot::init(int YL, int YR, int RL, int RR, bool load_calibration, int NoiseSensor, int Buzzer, int USTrigger, int USEcho)
{

  servo_pins[0] = YL;
  servo_pins[1] = YR;
  servo_pins[2] = RL;
  servo_pins[3] = RR;

  attachServos();

  isResting=false;

  if (load_calibration)
  {
    for (int i = 0; i < 4; i++)
    {
      int servo_trim = EEPROM.read(i);
      if (servo_trim > 128) servo_trim -= 256;
      servo[i].SetTrim(servo_trim);
    }
  }

  // Initialisation du capteur ultrason

  us.init(USTrigger, USEcho);

  // Buzzer et Pin de bruit
  pinBuzzer = Buzzer;
  pinMode(Buzzer, OUTPUT);

  pinNoiseSensor = NoiseSensor;
  pinMode(NoiseSensor, INPUT);

  // On met les servos à une position de départ.
  for(int i(0) ; i < 4 ; i++)
  {
    servo[i].SetPosition(90);
    servo_position[i] = 90;
  }

  delay(250);
}

/////////////////////////////////
/// Fonctions Attach et Detach //
/////////////////////////////////

void Kubot::attachServos()
{
      servo[0].attach(servo_pins[0]);
      servo[1].attach(servo_pins[1]);
      servo[2].attach(servo_pins[2]);
      servo[3].attach(servo_pins[3]);
}

void Kubot::detachServos()
{
      servo[0].detach();
      servo[1].detach();
      servo[2].detach();
      servo[3].detach();
}

//////////////////////////////////
/// Réglage des Trim des servo ///
//////////////////////////////////

void Kubot::setTrims(int YL, int YR, int RL, int RR)
{

  servo[0].SetTrim(YL);
  servo[1].SetTrim(YR);
  servo[2].SetTrim(RL);
  servo[3].SetTrim(RR);

}


void Kubot::saveTrimsOnEEPROM()
{
  for(int i(0); i < 4 ; i++)
    EEPROM.write(i, servo[i].getTrim());
}

/////////////////////////////////////
/// Mouvements basiques des servos///
/////////////////////////////////////

void Kubot::moveServos(int duration, int  servo_target[])
{

  setRestState(false);

  float increment[4];
  unsigned long finalTime;
  unsigned long partialTime;

  attachServos();


  if(duration > 10) // Si la durée est suffisament longue
  {
    // on calcul l'incremetation de chaque servo et le temps au quel les servos auront
    // fini de bouger
    // L'incrementation en temps est de 10ms
    for(int i(0); i < 4 ; i++)
      increment[i] = ((servo_target[i] - servo_position[i]))/(duration/10.f);

    finalTime = millis() + duration;

    for (int it(1) ; millis() < finalTime ; it++)
    {
      partialTime = millis() + 10;

      for (int j(0) ; j < 4 ; j++)
      {
        //servo_position[j] += it * increment[j];
        servo[j].SetPosition(servo_position[j] + it * increment[j]);
      }

      while (millis() < partialTime); // on attend qu'au moins 10 ms se soit écouler
    }

  }
  else //// si la durée est trop courte alors on bouge le plus vite possible
  {
    for(int i(0); i < 4 ; ++i)
    {
      servo[i].SetPosition(servo_target[i]);
      servo_position[i] = servo_target[i];
    }
  }

  // Une fois les mouvements effectué, on enregistre la position des servos
  for (int i = 0; i < 4; i++)
    servo_position[i] = servo_target[i];
}



void Kubot::oscillateServos(int A[4], int O[4], int T, double Ph[4], float cycle)
{
  for (int i(0); i<4; i++)
  {
    servo[i].SetO(O[i]);
    servo[i].SetA(A[i]);
    servo[i].SetT(T);
    servo[i].SetPh(Ph[i]);
  }

  float ref(millis());
  float x(ref);

  while(x <= T*cycle+ref)
  {
    for (int i(0); i < 4; i++)
    {
      servo[i].refresh();
    }
    x = millis();
  }

  for (unsigned i(0) ; i < 4 ; ++i)
  {
    //TODO: Recuper la position des servo a la fin du cycle
  }
}



void Kubot::oscillate(int A[4], int O[4], int T, double phase_diff[4], float steps)
{
  attachServos();
  setRestState(false);

  float cycles = trunc(steps);

  if(steps >= 1)
  {
    for(int i(0) ; i < cycles; ++i)
      oscillateServos(A,O,T,phase_diff);
  }

  oscillateServos(A,O,T,phase_diff,steps-cycles);

}





////////////////////////////////////////////////
/// Homing: Controle de la position de repos ///
////////////////////////////////////////////////


// La position de repos (tout les servos à 90)
// En prime, on désactive les servos par soucis
// d'économie d'énergie
void Kubot::home()
{

  if(!isResting)
  {

    int homes[4] = {90,90,90,90};
    moveServos(500,homes);

    setRestState(true);

    // Desactivation des servos.
    detachServos();

  }
}

bool Kubot::getRestState() {return isResting;}
void Kubot::setRestState(bool state) { isResting = state;}



//////////////////////////////////////////////
/// Capteurs: Ultrason et capteur de bruit ///
//////////////////////////////////////////////



// Utilisation du capteur ultrason
float Kubot::getDistance()
{
  return us.read();
}



// On recupere le bruit sur une pin vide. Celà permet de generer des nombres
// aléatoires, etc.
int Kubot::getNoise()
{

  int readingCount(2); // Le nombre de mesures à effectuer toujours > 0
  int noiseLevel(0);

  for (unsigned i(0) ; i < readingCount ; ++i)
    noiseLevel += analogRead(pinNoiseSensor);

  return noiseLevel / readingCount;

}



//////////////////////////////
/// Son: Gestion du buzzer ///
//////////////////////////////

// Permet au Kubot de jouer une certaine frequence pendant un temps donné
void Kubot::_tone(float frequency, long noteDuration, long silenceDuration)
{

  tone(pinBuzzer,frequency,noteDuration);

  // On attend que la note se termine...
  delay(noteDuration);

  // On attend au minimum 1ms avant l'execution d'autres commandes
  if(silenceDuration < 1)
    silenceDuration = 1;

  delay(silenceDuration);

}



// Permet au Kubot de jouer une note courbée
// dans la première itération, le temps par note est fixé à 15ms
void Kubot::bendTones(float initFrequency, float finalFrequency, float _step)
{
    bool ascending = (finalFrequency > initFrequency);

    // la pente permet de définir comment la note final sera atteinte.
    float slope = fabs(finalFrequency - initFrequency) / 100.f;

    if (ascending) // Si la frequence final est supérieure
    {
      for (int i(0) ; i <= 100 ; i += _step)
        _tone(initFrequency + slope*i, 15);
    }
    else
    {
      for (int i(100) ; i >= 0 ; i -= _step)
        _tone(initFrequency + slope*i, 15);
    }




}


// End of Kubot.cpp

#ifndef Kubot_hpp
#define Kubot_hpp

#include <Math.h>

#include <Servo.h>
#include <Oscillator.h>
#include <EEPROM.h>

#include <US.h>
#include <BatReader.h>


//-- Constantes
#define FORWARD     1
#define BACKWARD    -1
#define LEFT        1
#define RIGHT       -1
#define SMALL       5
#define MEDIUM      15
#define BIG         30

#define PIN_Buzzer  10
#define PIN_Trigger 8
#define PIN_Echo    9
#define PIN_NoiseSensor A6


// Classe Kubot.
// Cette classe definit un robot Kubot.
// Elle permet de le controler
class Kubot
{
  public:

    //-- Initialisation du Kubot.
    void initialize(int YL, int YR, int RL, int RR, bool load_calibration=true,
                    int NoiseSensor=PIN_NoiseSensor, int Buzzer=PIN_Buzzer,
                    int USTrigger=PIN_Trigger, int USEcho=PIN_Echo);

    //-- Fonctions Attach & detach
    void attachServos();
    void detachServos();

    //-- Réglage des Trim pour Oscillator
    void setTrims(int YL, int YR, int RL, int RR);
    void saveTrimsOnEEPROM();

    //-- Contrôle basique des servos
    void moveServos(int duration, int  servo_target[]);
    void oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle = 1.0);

    //-- HOME = Kubot à sa position de repos
    void home();
    bool getRestState();
    void setRestState(bool state);


    //-- Fonctions de capteurs
    float getDistance(); //Capteur ultrasons
    int getNoise();      //Capteur de bruit

    //-- Son
    void _tone(float frequency, long noteDuration, long silenceDuration = 1);
    void bendTones(float initFrequency, float finalFrequency, float _step);

  private:

    // Cette fonction est utilisé seulement pour les mouvements pré-programmés...
    // Elle permet de raisonner en terme de pas et définir l'état du Kubot.
    // TODO: Peut-être mergé cette fonction avec oscillateServos
    void oscillate(int A[4], int O[4], int T, double phase_diff[4], float steps = 1.0);

  private:

    BatReader battery;
    Oscillator servo[4];
    US us;

    int servo_pins[4];
    int servo_position[4];

    int pinBuzzer;
    int pinNoiseSensor;

    float increment[4];

    bool isResting;



};

#endif // Kubot_hpp

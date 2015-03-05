/*----------------------------------------------------------------
 * Application de pilotage des aiguilles avec 8 servomoteurs
 * et 8 boutons
 * Affectation des broches :
 * Servos 0 à 7 sur broches 0 à 7
 * LED sur broches 8 à 12
 * Clavier analogique de commande des servos sur broche A0
 * Clavier analogique de réglage sur broche A1
 * Codeur de désignation du servo à réglé sur broches A2 à A5
 *
 * Ce logiciel est distribuée sous licence GPL V2.
 *
 * L'utilisation commerciale est autorisée à condition de
 * de distribuer le code source de l'application à l'utilisateur
 *
 * © 2013 - Jean-Luc Béchennec
 */
#include <Servo.h>
#include <EEPROM.h>

const byte SERVO_A_ANGLE_MIN = 0;
const byte SERVO_A_ANGLE_MAX = 1;
const byte SERVO_EN_MOUVEMENT_VERS_ANGLE_MAX = 2;
const byte SERVO_EN_MOUVEMENT_VERS_ANGLE_MIN = 3;
 
const int ANGLE_MIN = 1000;
const int ANGLE_MAX = 2000;

const byte NON_PRESSE = 0;
const byte ENFONCE = 1;
const byte PRESSE = 2;
  
const byte AUCUN_EVENEMENT = 0;
const byte EVENEMENT_PRESSE = 1;
const byte EVENEMENT_RELACHE = 2;

/*
 * Classe de gestion d'un clavier analogique
 */
class DescripteurClavierAnalogique {
    byte etatAutomate;
    int  etatPoussoir;
    int  taillePlage;
    int  nbPoussoirs;
    int  pin;
 
    int lirePoussoirs()
    {
        int resultat;
        int numPoussoir = (analogRead(pin) + taillePlage / 2) / taillePlage;
        
        int nouvelEtatPoussoir = etatPoussoir; /* à priori rien ne change */
     
        switch (etatAutomate) {
            case NON_PRESSE:
                if (numPoussoir < nbPoussoirs)
                    etatAutomate = ENFONCE;
                break;
            case ENFONCE:
                if (numPoussoir < nbPoussoirs) {
                    etatAutomate = PRESSE;
                    nouvelEtatPoussoir = numPoussoir;
                }
                else {
                    etatAutomate = NON_PRESSE;
                }
                break;
            case PRESSE:
                if (numPoussoir == nbPoussoirs) {
                    etatAutomate = NON_PRESSE;
                    nouvelEtatPoussoir = -1;
                }
                break;
        }
    
        return nouvelEtatPoussoir;
    }
 
  public:
  
    DescripteurClavierAnalogique()
    {
        etatAutomate = NON_PRESSE;
        etatPoussoir = -1;
    }
    
    void connecte(int pinDeConnexion, int nombreDePoussoirs)
    {
        pin = pinDeConnexion;
        nbPoussoirs = nombreDePoussoirs;
        /* 
         * calcule la taille d'une plage
         * le convertisseur a 1024 valeurs, Le taille de plage
         * est égal à 1024/nombreDePoussoirs.
         */
        taillePlage = 1024 / nombreDePoussoirs;
    }

    /*
     * construction d'un événement en comparant
     * le nouvel état des poussoirs avec l'état précédent.
     */
    byte lireEvenement(int *numPoussoir)
    {
        byte evenement;
        int nouvelEtatPoussoir = lirePoussoirs();
            
        if (nouvelEtatPoussoir == etatPoussoir)
            evenement = AUCUN_EVENEMENT;
        if (nouvelEtatPoussoir >= 0 && etatPoussoir == -1) 
            evenement = EVENEMENT_PRESSE;
        if (nouvelEtatPoussoir == -1 && etatPoussoir >= 0) 
            evenement = EVENEMENT_RELACHE;
     
        etatPoussoir = nouvelEtatPoussoir;
        *numPoussoir = etatPoussoir;
        
        return evenement;
    }
};

DescripteurClavierAnalogique clavierOrdreServo;
DescripteurClavierAnalogique clavierReglage;


/*
 * Gestion des témoins en Charlieplexing
 */
enum { OUT0, OUT1, INZ };

const byte etatSortiePourLED[16][5] = {
  { OUT0, OUT1, INZ, INZ, INZ }, /* D0 */
  { OUT1, OUT0, INZ, INZ, INZ }, /* D1 */
  { INZ, OUT0, OUT1, INZ, INZ }, /* D2 */
  { INZ, OUT1, OUT0, INZ, INZ }, /* D3 */
  { INZ, INZ, OUT0, OUT1, INZ }, /* D4 */
  { INZ, INZ, OUT1, OUT0, INZ }, /* D5 */
  { INZ, INZ, INZ, OUT0, OUT1 }, /* D6 */
  { INZ, INZ, INZ, OUT1, OUT0 }, /* D7 */
  { OUT0, INZ, OUT1, INZ, INZ }, /* D8 */
  { OUT1, INZ, OUT0, INZ, INZ }, /* D9 */
  { INZ, INZ, OUT0, INZ, OUT1 }, /* D10 */
  { INZ, INZ, OUT1, INZ, OUT0 }, /* D11 */
  { INZ, OUT0, INZ, OUT1, INZ }, /* D12 */
  { INZ, OUT1, INZ, OUT0, INZ }, /* D13 */
  { OUT0, INZ, INZ, INZ, OUT1 }, /* D14 */
  { OUT1, INZ, INZ, INZ, OUT0 }  /* D15 */
};

enum { ALLUME, ETEINT };
 
byte etatLED[16] = {
    ETEINT, ETEINT, ETEINT, ETEINT,
    ETEINT, ETEINT, ETEINT, ETEINT,
    ETEINT, ETEINT, ETEINT, ETEINT,
    ETEINT, ETEINT, ETEINT, ETEINT
};

void programmeBroche(int numBroche, byte etat)
{
  switch (etat) {
    case OUT0:
      digitalWrite(numBroche,LOW);
      pinMode(numBroche,OUTPUT);
      break;
    case OUT1:
      digitalWrite(numBroche,HIGH);
      pinMode(numBroche,OUTPUT);
      break;
    case INZ:
      pinMode(numBroche,INPUT);
      digitalWrite(numBroche,LOW); /* pas de pullup */
      break;
  }
}

void gereLED(int num)
{
  if (etatLED[num] == ALLUME) {
    programmeBroche(8,etatSortiePourLED[num][0]);
    programmeBroche(9,etatSortiePourLED[num][1]);
    programmeBroche(10,etatSortiePourLED[num][2]);
    programmeBroche(11,etatSortiePourLED[num][3]);
    programmeBroche(12,etatSortiePourLED[num][4]);
  }
  else {
    programmeBroche(8,INZ);
    programmeBroche(9,INZ);
    programmeBroche(10,INZ);
    programmeBroche(11,INZ);
    programmeBroche(12,INZ);
  }
}

/*
 * Gestion des servomoteurs
 */
boolean reglageEnCours = false;
byte pasDeReglage = 8;

/*
 * Descripteur d'un servomoteur
 */
class DescripteurServo {
    Servo objetServo;      /* Objet issu de la bibliothèque Servo de l'Arduino */
    int vitesse;           /* La vitesse de déplacement du servomoteur */
    int angle;             /* L'angle du servo-moteur */
    int angleMin;          /* Angle minimum du servo */
    int angleMax;          /* Angle maximum du servo */
    int adresse;           /* Adresse en EEPROM pour sauver angleMin et angleMax */
    int pin;               /* La broche sur laquelle il est connecté */
    int numServo;          /* Numéro du servomoteur */
    int cptArret;          /* Temporisation avant de détacher le servo */
    byte etatServo;        /* son état : SERVO_A_ANGLE_MIN, SERVO_A_ANGLE_MAX
                              SERVO_EN_MOUVEMENT_VERS_ANGLE_MAX ou
                              SERVO_EN_MOUVEMENT_VERS_ANGLE_MIN    */
    boolean aSauver;       /* Vrai si on doit enregistrer en EEPROM les angles
                              du servomoteurs */

  public:
    /* Constructeur, le servo est mis arreté à l'angle min */
    DescripteurServo()
    {
        angleMin = ANGLE_MIN;
        angleMax = ANGLE_MAX;
        angle = angleMin;
        vitesse = 0;
        cptArret = -1;
        etatServo = SERVO_A_ANGLE_MIN;
        aSauver = false;
    }
    
    /*
     * Connexion, le servo est connecté à une broche et asservi
     * On relit les valeur d'angles réglées dans l'EEPROM
     */
    void connecte(int numeroServo, int pinDeConnexion)
    {
        adresse = numeroServo * 4;
        pin = pinDeConnexion;
        numServo = numeroServo;
        
        angleMin = EEPROM.read(adresse) | ((int)EEPROM.read(adresse + 1)) << 8;
        angleMax = EEPROM.read(adresse + 2) | ((int)EEPROM.read(adresse + 3)) << 8;
        if (angleMin < ANGLE_MIN || angleMin > ANGLE_MAX) angleMin = ANGLE_MIN;
        if (angleMax < ANGLE_MIN || angleMax > ANGLE_MAX) angleMax = ANGLE_MAX;
        
        angle = angleMin;
        /* allume la LED correspondante */
        etatLED[numServo * 2] = ALLUME;
        
        objetServo.attach(pinDeConnexion);
        cptArret = 50;
        objetServo.writeMicroseconds(angle);
    }
  
    void arreteServo()
    {
        vitesse = 0;
        cptArret = 30; 
    }
    
    /*
     * Ajoute la vitesse à l'angle. Stoppe le servo quand l'angle min ou max
     * est atteint. Stoppe son asservissement
     * Doit etre appelé péridioquement à un intervalle de temps fixe
     */
    void gereServo()
    {
        objetServo.writeMicroseconds(angle);
     
        if (vitesse == 0)
            if (cptArret == 0)
                objetServo.detach();
            if (cptArret >= 0)
                cptArret--;
        else
            angle += vitesse;
        
        if (! reglageEnCours ||
            etatServo == SERVO_EN_MOUVEMENT_VERS_ANGLE_MIN ||
            etatServo == SERVO_EN_MOUVEMENT_VERS_ANGLE_MAX) {
            if (angle > angleMax) {
                angle = angleMax;
                arreteServo();
                etatServo = SERVO_A_ANGLE_MAX;
                etatLED[numServo * 2 + 1] = ALLUME;
            }
            else if (angle < angleMin) {
                angle = angleMin;
                arreteServo();
                etatServo = SERVO_A_ANGLE_MIN;
                etatLED[numServo * 2] = ALLUME;
            }
        }
        else {
            if (etatServo == SERVO_A_ANGLE_MIN) {
                if (vitesse > 0 && angle >= angleMin) {
                    angle = angleMin;
                    arreteServo();
                }
                else if (vitesse < 0 && angle <= angleMin) {
                    angle = angleMin;
                    arreteServo();
                }
            }
            else if (etatServo == SERVO_A_ANGLE_MAX) {
                if (vitesse > 0 && angle >= angleMax) {
                    angle = angleMax;
                    arreteServo();
                }
                else if (vitesse < 0 && angle <= angleMax) {
                    angle = angleMax;
                    arreteServo();
                }
            }
        }
    }
    
    /*
     * Indique au servo qu'il doit gagner la position opposée à
     * la position actuelle
     */
    void evenementServo()
    {
        cptArret = -1;
        switch (etatServo) {
          case SERVO_A_ANGLE_MIN:
            etatLED[numServo * 2] = ETEINT;
            objetServo.attach(pin);
          case SERVO_EN_MOUVEMENT_VERS_ANGLE_MIN:
            vitesse =  1;
            etatServo = SERVO_EN_MOUVEMENT_VERS_ANGLE_MAX;
            break;
          case SERVO_A_ANGLE_MAX:
            etatLED[numServo * 2 + 1] = ETEINT;
            objetServo.attach(pin);
          case SERVO_EN_MOUVEMENT_VERS_ANGLE_MAX:
            vitesse = -1;
            etatServo = SERVO_EN_MOUVEMENT_VERS_ANGLE_MIN;
            break;
        }
    }
    
//    void print()
//    {
//      Serial.print("*** angleMin = "); Serial.println(angleMin);
//      Serial.print("*** angleMax = "); Serial.println(angleMax);
//      Serial.print("*** angle = "); Serial.println(angle);
//      Serial.print("*** vitesse = "); Serial.println(vitesse);
//      Serial.print("*** etat = "); 
//      switch (etatServo) {
//        case SERVO_A_ANGLE_MIN : Serial.println("*** ANGLE_MIN"); break;
//        case SERVO_A_ANGLE_MAX : Serial.println("*** ANGLE_MAX"); break;
//        case SERVO_EN_MOUVEMENT_VERS_ANGLE_MIN : Serial.println("*** MOUVEMENT_VERS_ANGLE_MIN"); break;
//        case SERVO_EN_MOUVEMENT_VERS_ANGLE_MAX : Serial.println("*** MOUVEMENT_VERS_ANGLE_MAX"); break;
//      }
//    }
    
    /*
     * Regle un servo moteur
     */
    void regleServo()
    {
        int numPoussoir;
        
        if (clavierReglage.lireEvenement(&numPoussoir) == EVENEMENT_PRESSE) {
            switch(numPoussoir) {
              
              case 0: /* déplacement de la butée dans le sens trigo */
                switch(etatServo) {
                  case SERVO_A_ANGLE_MIN:
                    if (angleMin < angleMax)
                      angleMin += min(pasDeReglage, angleMax - angleMin);
                    objetServo.attach(pin);
                    vitesse = 1;
                    aSauver = true;
                     break;
                  case SERVO_A_ANGLE_MAX:
                    if (angleMax < ANGLE_MAX)
                      angleMax += min(pasDeReglage, ANGLE_MAX - angleMax);
                    objetServo.attach(pin);
                    vitesse = 1;
                    aSauver = true;
                    break;
                }
                break;
                
              case 1: /* déplacement de la butée dans le sens horaire */
                switch(etatServo) {
                  case SERVO_A_ANGLE_MIN:
                    if (angleMin > ANGLE_MIN)
                      angleMin -= min(pasDeReglage, angleMin - ANGLE_MIN);
                    objetServo.attach(pin);
                    vitesse = -1;
                    aSauver = true;
                    break;
                  case SERVO_A_ANGLE_MAX:
                    if (angleMax > angleMin)
                      angleMax -= min(pasDeReglage, angleMax - angleMin);
                    objetServo.attach(pin);
                    vitesse = -1;
                    aSauver = true;
                    break;
                }
                break;
                
              case 2: /* diminution du pas de réglage */
                if (pasDeReglage > 1) pasDeReglage /= 2;
                break;
                
              case 3: /* augmentation du pas de réglage */
                if (pasDeReglage < 32) pasDeReglage *= 2;
                break;
            }
        }
    }
    
    void ecrireEEPROMSiDifferent(int adresseEcriture, byte valeur)
    {
        if (EEPROM.read(adresseEcriture) != valeur)
            EEPROM.write(adresseEcriture, valeur);
    }
    
    void enregistre()
    {
        if (aSauver) {
            ecrireEEPROMSiDifferent(adresse, angleMin & 0xFF);
            ecrireEEPROMSiDifferent(adresse + 1, angleMin >> 8);
            ecrireEEPROMSiDifferent(adresse + 2, angleMax & 0xFF);
            ecrireEEPROMSiDifferent(adresse + 3, angleMax >> 8);
            aSauver = false;
        }            
    }
};
 
DescripteurServo servoMoteur[8];

/*
 * Lecture du codeur de réglage.
 * Retourne -1 si aucun servo n'est sélectionné et
 * le numéro du servo de 0 à 7 si un servo est sélectionné
 */
int lireReglage()
{
    int servoARegler =
        (! digitalRead(A2)) << 3 |
        (! digitalRead(A3)) << 2 | 
        (! digitalRead(A4)) << 1 |
        (! digitalRead(A5));
    
    if (servoARegler > 8) servoARegler = 0;
    
    return servoARegler - 1;
}

void initialiseCodeur()
{
    /* Active le pullup des entrées A2, A3, A4 et A5 */
    digitalWrite(A2,HIGH);
    digitalWrite(A3,HIGH);
    digitalWrite(A4,HIGH);
    digitalWrite(A5,HIGH);
}

void setup()
{
    /* Initialisation des servos */
    int numServo;
  
    for (numServo = 0; numServo < 8; numServo++)
        servoMoteur[numServo].connecte(numServo, numServo);
        
    clavierOrdreServo.connecte(A0,8); /* pin A0, 8 poussoirs */
    clavierReglage.connecte(A1,4);    /* pin A1, 4 poussoirs */
    
    /* Initialisation pour le codeur */
    initialiseCodeur();
}

int ancienServoARegler = -1; /* aucun */

int numLED = 0;
int compteurGestionServos = 0;

void loop()
{
    compteurGestionServos++;
    
    if (compteurGestionServos == 5) {
        
        compteurGestionServos = 0;
      
        int numServo;
    
        /* gestion du mouvement des servos */
        for (numServo = 0; numServo < 8; numServo++)
            servoMoteur[numServo].gereServo();
     
        /* lecture d'un ordre de mouvement */
        byte evenement = clavierOrdreServo.lireEvenement(&numServo);
      
        /* exécution de l'ordre de mouvement */
        if (evenement == EVENEMENT_PRESSE)
            servoMoteur[numServo].evenementServo();
        
        /* lecture du codeur qui désigne le servo à régler */
        int servoARegler = lireReglage();
        /* si positif ou nul, il y a un servo à régler */
        reglageEnCours = (servoARegler >= 0);
        /* si on a changé de servo à régler, le pas est remis à 8 */
        if (servoARegler != ancienServoARegler) {
          ancienServoARegler = servoARegler;
          pasDeReglage = 8;
        }
    
        if (reglageEnCours)
            /* exécution du réglage */
            servoMoteur[servoARegler].regleServo();
        else {
            /* enregistrement en EEPROM des servos qui ont été réglés */
            for (numServo = 0; numServo < 8; numServo++)
                servoMoteur[numServo].enregistre();
        }
    }
  
    gereLED(numLED);
 
    numLED++;
    if (numLED == 16)
        numLED = 0;
  
    delay(1);
}

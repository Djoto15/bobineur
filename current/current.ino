#include <Wire.h>
#include <Adafruit_INA219.h>

// Instanciation de l'INA219 avec son adresse I2C matérielle (0x45)
Adafruit_INA219 ina219(0x45);
//dafruit_INA219 ina219;

const int PWM_PIN = 9;

// Consigne de courant initiale en mA (ex: 40 mA pour le test avec la résistance 220 Ohms)
float consigneCourant_mA = 40.0;
float courantFiltre_mA = 0.0;

// Coefficients du correcteur PI (doux pour éviter l'instabilité et les oscillations)
const float Kp = 0.05;
const float Ki = 0.01;
float integral = 0.0;
float rapportCyclique = 0.0;

unsigned long dernierTemps = 0;
const unsigned long INTERVALLE_MS = 200; // Période d'échantillonnage de 400 ms (2.5 Hz)

void ina219WriteConfig(uint16_t cfg){
  Wire.beginTransmission(0x45);
  Wire.write(0x00);
  Wire.write(cfg >> 8);
  Wire.write(cfg & 0xFF);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  //Serial.setTimeout(20); // Timeout court pour ne pas bloquer la boucle lors de la saisie série

  // Configuration du Timer 1 pour générer un PWM rapide à 31.25 kHz sur D9
  // (permet de lisser parfaitement le courant dans la charge et supprime tout sifflement audible)
  //TCCR1A = _BV(COM1A1) | _BV(WGM10);
  //TCCR1B = _BV(CS10);

  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);

  if (!ina219.begin()) {
    Serial.println(F("Erreur : INA219 non detecte a l'adresse 0x45 !"));
    while (1);
  }

  // Configure l'INA219 pour moyenner 32 mesures en interne sur le shunt
  // Réduit drastiquement le bruit sans ralentir la boucle
  // 0x391F = Plage 32V, Gain /8 (320mV), Shunt ADC: 12-bit 16 samples (~8.5 ms)
  ina219WriteConfig(0x39EF);

  Serial.println(F("Etage de puissance initialise"));
  Serial.println(F("Entre une consigne en mA (ex: 40) :"));
}

void loop() {
  // 1. Lecture d'une éventuelle nouvelle consigne envoyée via le moniteur / traceur série
  if (Serial.available() > 0) {
    float nouvelleConsigne = Serial.parseFloat();
    
    // Purge des caractères résiduels dans le buffer (\r, \n, espaces)
    // afin d'éviter que le retour chariot ne réinitialise la consigne à 0.0
    while (Serial.available() > 0) {
      Serial.read();
    }
    
    if (nouvelleConsigne >= 0.0 && nouvelleConsigne <= 1000.0) {
      consigneCourant_mA = nouvelleConsigne;
      integral = 0.0; // Réinitialise l'intégrateur pour éviter tout à-coup
    }
  }

  // 2. Boucle de régulation à cadence fixe (20 ms)
  unsigned long maintenant = millis();
  if (maintenant - dernierTemps >= INTERVALLE_MS) {
    dernierTemps = maintenant;

    /*
     * EXPLICATION DU FACTEUR * 10.0 :
     * La bibliothèque Adafruit_INA219 est configurée en dur pour une résistance shunt standard
     * de 0.1 Ohm (marquée R100). Or, le module DFRobot SEN0291 intègre une résistance shunt
     * de 0.01 Ohm (marquée R010) pour pouvoir mesurer des courants plus élevés.
     * Comme I = U_shunt / R_shunt et que le shunt réel est 10 fois plus petit que celui
     * attendu par le calcul interne de la bibliothèque, la valeur renvoyée est 10 fois trop faible.
     * Multiplier la lecture par 10.0 rétablit la valeur physique exacte en mA.
     */
    float mesureBrute = 0;
    mesureBrute = ina219.getCurrent_mA() * 10.0;
    if (mesureBrute < 0.0) mesureBrute = 0.0;

    // Filtre passe-bas exponentiel pour éliminer le bruit de hachage et l'aliasing
    courantFiltre_mA = (0.90 * courantFiltre_mA) + (0.10 * mesureBrute);

    if (consigneCourant_mA <= 0.0) {
      rapportCyclique = 0.0;
      integral = 0.0;
    } else {
      float erreur = consigneCourant_mA - courantFiltre_mA;

      // Accumulation intégrale avec borne anti-emballement (anti-windup)
      integral += erreur * (INTERVALLE_MS / 1000.0);
      integral = constrain(integral, 0.0, 100.0);

      // Calcul de la correction PI
      float commande = (Kp * erreur) + (Ki * integral);
      
      // Mise à jour progressive du rapport cyclique (0 à 255)
      rapportCyclique = constrain(rapportCyclique + commande, 0.0, 255.0);
    }

    // Application du signal PWM sur la grille du MOSFET via la broche D9
    analogWrite(PWM_PIN, (int)rapportCyclique);

    // Télémétrie au format compatible avec le Traceur série (Serial Plotter)
    Serial.print(F("Consigne:"));
    Serial.print(consigneCourant_mA);
    Serial.print(F(",Courant:"));
    Serial.print(mesureBrute);
    Serial.print(F(",Courant filtre:"));
    Serial.print(courantFiltre_mA);
    Serial.print(F(",PWM:"));
    Serial.println((int)rapportCyclique);
  }
}
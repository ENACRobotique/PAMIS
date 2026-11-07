# Composant `stepper`

Ce composant ESP-IDF pilote un moteur pas à pas par les signaux **STEP**, **DIR**
et **EN**. La classe [`Stepper`](include/stepper.h) utilise un GPTimer à
1 MHz et une interruption par impulsion pour exécuter un déplacement relatif
avec accélération, palier éventuel et décélération.

L'implémentation est dans [`stepper.cpp`](stepper.cpp), et son intégration aux
deux moteurs du robot dans [`main/locomotion.cpp`](../../main/locomotion.cpp).
La position comptabilise les impulsions émises : aucun retour de position moteur
n'est utilisé pour détecter une perte de pas.

## Configuration et unités

`config()` copie une structure `Stepper_config_t`, puis `init()` configure les
GPIO, crée le sémaphore de fin de mouvement et le timer. La surcharge
`init(stepPin, dirPin, enPin, microstep)` permet aussi de définir les broches.
EN est actif à l'état bas. Le réglage `miStep` sert aux conversions de distance ;
il ne configure pas le microstepping du driver, qui doit être réglé séparément.

Dans cette documentation, un **pas interne** correspond à une impulsion STEP,
donc à un micro-pas lorsque le driver utilise le microstepping.

| Grandeur | Unité / signification |
| --- | --- |
| Argument de `setStepsPerMm(S)` | Pas entiers du moteur par mm, avant microstepping |
| `ctrl.stepsPerMm = S × miStep` | Impulsions STEP par mm |
| `setSpeedMm(speed, acc)` | `speed` en mm/s ; `acc` en mm/s² |
| `ctrl.v_max` | Impulsions/s |
| `ctrl.acc` | Impulsions/s² |
| `ctrl.T`, `ctrl.T_min` | Périodes en ticks de timer, codées en virgule fixe Q18.14 |
| `TIMER_F` | 1 000 000 ticks/s, soit 1 µs par tick |

Pour une roue de diamètre `D` en mm et un moteur d'angle de pas `θ` en degrés :

```text
S = (360 / θ) / (π × D)
K = S × miStep              // impulsions par mm
v_max = speed × K
a = acc × K
```

Le freinage utilise la même accélération `acc` que la montée en vitesse.

## Calcul de la pente d'accélération

### 1. Choisir l'accélération physique

La pente est celle de la **vitesse en fonction du temps** :

```text
a_mm = Δv_mm / Δt
```

Pour passer du repos à 100 mm/s en 0,1 s, demander `acc = 1000` mm/s².
Après conversion en pas internes, `a = 1000 × K`.
Pour une accélération constante depuis le repos, les relations théoriques sont :

```text
v(t) = a × t
x(t) = a × t² / 2
t_acc = v_max / a
pas_acc_théorique = v_max² / (2 × a)
```

La période entre impulsions ne diminue donc **pas linéairement** : la vitesse
est l'inverse de cette période. Soustraire un délai constant à chaque pas ne
donnerait pas une accélération constante.

### 2. Passer de la trajectoire aux périodes

Le pas numéro `n` est théoriquement atteint à l'instant :

```text
t_n = sqrt(2 × n / a)
Δt_n = t_n - t_(n-1)
     = sqrt(2 / a) × (sqrt(n) - sqrt(n-1))
P_n = TIMER_F × Δt_n                  // période en ticks
```

La période théorique du premier pas vaut `TIMER_F × sqrt(2 / a)`. Elle ne peut
cependant pas être utilisée directement comme point de départ de la récurrence,
car les petites erreurs de chaque mise à jour s'accumuleraient. `runPos()`
applique le facteur correctif `0,676` :

```cpp
ctrl.n = 1;
ctrl.T = (uint32_t)Q(0.676 * sqrt(2.0 / ctrl.acc) * TIMER_F);
ctrl.T_min = (uint32_t)Q(TIMER_F / ctrl.v_max);
```

`T_min` est la période correspondant à la vitesse maximale demandée.
Les expressions ci-dessus décrivent les conversions ; le code limite les périodes entre 1 et 100 000 µs avant conversion et impose ensuite `T >= T_min` au démarrage.

### 3. Mettre à jour la période sans racine carrée dans l'interruption

Au lieu d'évaluer les racines à chaque impulsion, `xISR()` utilise la récurrence
approchée suivante pendant l'état `ACC` :

```cpp
ctrl.T -= (2 * ctrl.T) / (4 * ctrl.n + 1);
if (ctrl.T < ctrl.T_min) {
    ctrl.T = ctrl.T_min;
}
ctrl.n++;
```

En notant `T_n` la période avant cette mise à jour :

```text
T_(n+1) ≈ T_n × (1 - 2 / (4n + 1))
```

Le terme retranché devient progressivement plus petit. À grand `n`, la formule
exacte donne une période proportionnelle à `1 / sqrt(n)` ; sa variation relative
est approximativement `-1 / (2n)`, ce qui explique la forme de la récurrence.
Celle-ci évite les racines carrées dans l'interruption et conserve une diminution
progressive de la période.

**Cette récurrence reste une approximation du profil physique.** Sans
correction, la première mise à jour donnerait `T_2 = 0,6 × T_1`, alors que la
formule exacte donne `T_2 = (sqrt(2) - 1) × T_1 ≈ 0,4142 × T_1`. Le facteur
`0,676` place la récurrence beaucoup plus près des périodes théoriques sur
l'ensemble de la rampe. La borne `T_min` empêche ensuite de dépasser la vitesse
maximale demandée.

Cette correction vient de l'algorithme publié par David Austin. La note
d'application AVR446 d'Atmel/Microchip en reprend la dérivation et indique
explicitement, dans la section 2.3.1, de multiplier la période initiale par
`0,676` pour compenser l'erreur de l'approximation au premier pas.

### 4. Conserver les fractions de tick

Les macros `Q` et `F` convertissent entre ticks et représentation Q18.14 :

```text
Q(P) = P × 16384 + 0,5       // conversion vers uint32_t ensuite
F(T) = T / 16384            // division entière pour programmer le timer
```

La récurrence travaille sur `ctrl.T` avec 14 bits de fraction. Cela conserve
des diminutions inférieures à un tick, qui seraient perdues en calculant
directement sur des périodes entières. L'alarme reçoit finalement
`alarm_cfg.alarm_count = F(ctrl.T)` : sa résolution reste de 1 µs.
La résolution interne est de 1/16384 µs, soit 0,061035 ns ; les calculs de la
récurrence restent sur 32 bits.

### 5. Répartir accélération, palier et décélération

Pour `N = abs(nb_steps)`, `runPos()` calcule :

```text
n_vmax = floor(v_max² / (2 × a))

si 2 × n_vmax < N :
    n_ramp = n_vmax       // profil prévu trapézoïdal
sinon :
    n_ramp = floor(N / 2) // profil prévu triangulaire
```

Dans le modèle idéal, un mouvement trop court pour atteindre la consigne a une
vitesse de pointe `sqrt(2 × a × n_ramp)`. Pour un profil trapézoïdal, la longueur
théorique du palier vaut `N - 2 × n_ramp`.

L'état initial vaut toujours `ACC`, même si `n_ramp` est nul.
Après chaque impulsion, l'état décrit la phase du prochain pas :

1. Si c'est le dernier pas, arrêter le timer et passer à `IDLE` sans recalculer
   de période ni décrémenter `n`.
2. Sinon, choisir la phase du prochain pas : `DEC` si `steps_left <= n_ramp`,
   sinon `COAST` si `steps_done >= n_ramp`, sinon rester en `ACC`.
   Un freinage déjà engagé, notamment par `stop()`, reste en `DEC`.
3. Calculer immédiatement la période de ce prochain pas : la diminuer en
   `ACC`, la conserver en `COAST`, ou l'augmenter en `DEC` :

   ```cpp
   ctrl.T += (2 * ctrl.T) / (4 * ctrl.n + 1);
   ctrl.n--;
   ```

Un profil triangulaire pair passe directement de `ACC` à `DEC`. Pour un nombre
impair de pas, le pas central est en `COAST`, conformément au reste de la division
entière : `N - 2 × n_ramp = 1`. Un mouvement d'un seul pas termine sans mise à jour de période.
La récurrence de freinage reste une approximation, et n'est pas l'inverse
algébrique exact de celle d'accélération.

### Exemple numérique

Avec `S = 1` pas entier/mm, un microstepping de 16, une vitesse de 100 mm/s,
une accélération de 1000 mm/s² et un déplacement de 100 mm :

| Calcul | Résultat |
| --- | --- |
| `K = S × 16` | 16 impulsions/mm |
| `v_max = 100 × K` | 1600 impulsions/s |
| `a = 1000 × K` | 16 000 impulsions/s² |
| `N = 100 × K` | 1600 impulsions |
| `P_1 théorique = 10⁶ × sqrt(2 / a)` | Environ 11 180,34 ticks |
| `P_1 corrigée = 0,676 × P_1 théorique` | Environ 7557,91 ticks |
| `P_min = 10⁶ / v_max` | 625 ticks |
| `n_vmax = floor(v_max² / (2a))` | 80 impulsions, soit 5 mm |
| Profil prévu | Trapézoïdal : 80 / 1440 / 80 impulsions |

Les premières périodes de la récurrence, avant quantification du timer, valent
environ 7557,91 ; 4534,75 ; 3527,02 ticks. Le temps d'accélération théorique
est de 0,1 s ; le profil discret corrigé s'en approche sans le garantir exactement.

## Utilisation

Exemple avec les broches du moteur gauche du projet et un coefficient mécanique
illustratif de 1 pas entier/mm, à remplacer par celui de la transmission :

```cpp
#include "stepper.h"

static Stepper moteur; // doit rester vivant pendant les interruptions

void exemple()
{
    moteur.init(39, 38, 40, MICROSTEP_16);
    moteur.setStepsPerMm(1.0);
    moteur.setSpeedMm(100.0, 1000.0);

    ESP_ERROR_CHECK(moteur.runPosMm(100));
    if (moteur.waitFinishedTimeout(pdMS_TO_TICKS(5000)) != pdTRUE) {
        moteur.emergencyStop();
    }
}
```

Appeler l'initialisation une seule fois. Régler le coefficient mécanique avant
les vitesses et les déplacements en mm. Configurer les paramètres à l'arrêt.

| Méthode | Comportement |
| --- | --- |
| `runPos(int32_t)` | Déplacement relatif en impulsions ; positif = `CW`, négatif = `CCW`. Démarrage asynchrone, activation automatique si désactivé. |
| `runPosMm(int32_t)` | Déplacement relatif en mm entiers ; conversion en impulsions, puis troncature vers `int32_t`. |
| `waitFinishedTimeout(ticks)` | Attend la fin ; renvoie `pdTRUE` si terminé ou déjà inactif, sinon `pdFALSE` au délai. Un délai expiré n'arrête pas le moteur. |
| `emergencyStop()` | Arrêt immédiat du timer, sans rampe ; laisse le driver activé. |
| `stop()` | Depuis `ACC` ou `COAST`, raccourcit la cible et passe en freinage. Renvoie en mm signés la portion retranchée à la cible initiale, pas la distance de freinage. Renvoie zéro dans les autres états. |
| `stop(double braking_distance)` | Impose une distance maximale de freinage en mm, positive quel que soit le sens, y compris depuis `DEC`. Garde le freinage prévu s'il est plus court. Arrondit au micro-pas supérieur. Zéro déclenche un arrêt immédiat. Renvoie la distance retranchée en mm signés. |
| `enableMotor()` / `disableMotor()` | Commandent EN et fixent l'état à `IDLE` / `DISABLED`. À utiliser à l'arrêt : ces fonctions n'arrêtent pas le timer d'un mouvement actif. |
| `getPosition()` / `getPositionMm()` | Position comptée depuis l'initialisation ou `resetAbsolute()`, en impulsions / mm. La lecture 64 bits est protégée vis-à-vis de l'ISR. |
| `resetAbsolute()` | Remet seulement la position comptée à zéro, sans mouvement de référencement. La remise à zéro est protégée vis-à-vis de l'ISR. |
| `getState()` | Renvoie `DISABLED`, `IDLE`, `ACC`, `COAST` ou `DEC`. |
| `getStepsPerMm()` | Renvoie le coefficient avant microstepping, contrairement à `ctrl.stepsPerMm`. |
| `getSpeed()` / `getTargetSpeed()` | Renvoient tous deux la consigne en impulsions/s, convertie en `uint16_t`, pas une vitesse instantanée mesurée. |
| `getAcc()` | Renvoie `ctrl.acc`, commun à l’accélération et au freinage, en impulsions/s². |

Une commande de zéro pas renvoie immédiatement `ESP_OK`, même pendant un autre
mouvement. Une autre commande reçue pendant `ACC`, `COAST` ou `DEC` est refusée
avec `ESP_ERR_NOT_SUPPORTED` ; il n'y a pas de file de mouvements.

`n_ramp` est le nombre de pas prévu pour chacune des deux rampes du
mouvement complet. `stop()` appelle la surcharge protégée sans imposer de
distance maximale. En accélération, elle utilise directement `n` comme nombre
de pas de freinage ; au palier, elle utilise `n_ramp`. Cela évite une racine
carrée, un carré entier et leurs risques de débordement.

### Freinage sur une distance imposée

```cpp
float distance_retranchee = moteur.stop(20.0); // freiner sur 20 mm maximum,
                                             // à un micro-pas d'arrondi près
```

`stop(double braking_distance)` conserve la période courante et recale `n`
sur le nombre d'impulsions de freinage. La distance demandée est une borne
haute : si le freinage normal prévu ou le freinage déjà engagé est plus court,
sa cible est conservée. Sinon, la nouvelle cible est la position comptée
actuelle plus la distance demandée dans le sens du mouvement. Les distances
négatives ou non finies sont refusées avec un log,
sans modifier le mouvement ; la méthode renvoie alors zéro.

La cible et l'indice sont modifiés sous une section critique partagée avec
l'ISR. L'accélération configurée n'est pas modifiée. Une distance plus courte
rend la récurrence de décélération plus forte. Le nombre d'impulsions jusqu'à
l'arrêt est imposé, mais la rampe reste approximative et la dernière période
reste finie : cela ne garantit pas que le moteur physique puisse suivre un
freinage arbitrairement court. Aucun ajustement automatique à partir d'un
capteur d'obstacle n'est effectué par cette surcharge.

## Évolution vers une commande en vitesse

Pour ajouter une commande en vitesse, il faudrait introduire un mode distinct du
déplacement en position et une consigne de vitesse signée, modifiable pendant le
mouvement. Dans ce mode, `steps_total`, `steps_done` et `n_ramp` ne détermineraient
plus la fin : l'ISR continuerait à produire des impulsions, comparerait la période
courante à celle de la consigne et accélérerait ou ralentirait avec la pente
configurée jusqu'à les faire coïncider. Une consigne nulle demanderait une
décélération jusqu'à l'arrêt ; une inversion de signe devrait d'abord atteindre
la vitesse nulle, puis changer DIR et accélérer dans l'autre sens. La position
comptée resterait mise à jour à chaque impulsion. Il faudrait également protéger
la nouvelle consigne avec `control_lock`, définir la priorité entre commandes en
position et en vitesse, et adapter `stop()`, `emergencyStop()` et l'attente de fin :
une commande en vitesse n'a pas de fin naturelle tant qu'elle n'a pas reçu une
consigne nulle ou une demande d'arrêt.

## Limites de l'implémentation

- `setStepsPerMm()` vérifie que le coefficient après microstepping est fini
  et strictement positif. Sinon, il écrit un log et conserve la valeur configurée.
- `setSpeedMm()` vérifie que les valeurs converties de vitesse et
  d'accélération sont finies et strictement positives. Un coefficient pas/mm
  non renseigné produit des valeurs nulles, rejetées par ce même contrôle.
  En cas d'erreur, la méthode écrit un log et conserve les deux valeurs configurées.
- Un mouvement est limité à **1 000 000 impulsions STEP**, soit environ 75 m
  avec une roue de 76,2 mm en microstepping ×16. `runPos()` et `runPosMm()`
  refusent les déplacements plus longs avec `ESP_ERR_INVALID_ARG`.
  Cette limite garantit aussi que `4*n+1` tient sur 32 bits, même lorsque
  `stop(distance)` recale l'indice sur toute la distance restante.
- `periodToFixed()` ramène les périodes calculées entre **1 et 100 000 µs**,
  puis les convertit en Q18.14. La borne haute garantit que `2*T` tient sur
  32 bits. Elle correspond à 10 impulsions/s, soit environ 0,75 mm/s pour
  une roue de 76,2 mm en ×16. Une consigne plus lente est donc limitée ;
  une accélération très faible peut également voir sa période initiale limitée.
  La borne basse de 1 µs est une limite numérique, pas une garantie de fréquence
  réalisable par l'ISR ou le moteur. La première période reste au moins `T_min`.
- Pendant le freinage, une addition qui dépasserait la période maximale
  sature à cette limite : le ralentissement cesse, mais les pas sont décomptés.
- `runPosMm()` vérifie que le coefficient pas/mm est renseigné et que le
  déplacement converti respecte la limite, avant sa conversion entière.
- L'interruption utilise `gpio_ll_set_level()`, une fonction bas niveau inline
  qui sélectionne la banque GPIO et écrit dans les registres de mise à un ou à
  zéro. La broche STEP doit être une sortie GPIO valide. Cette fonction appartient
  à la couche LL interne d'ESP-IDF.
- La largeur de l'impulsion STEP dépend du temps d'exécution de l'interruption ;
  aucun délai haut explicite n'est programmé.
- Dans le robot, les deux moteurs partagent EN sur GPIO 40 : désactiver l'un
  coupe physiquement les deux drivers, sans synchroniser leurs états logiciels.
- La surcharge `stop(distance)` est protégée vis-à-vis de l'interruption.
  Les autres accès aux variables `volatile` ne disposent pas d'une protection
  générale contre les accès concurrents. Les appels de commande doivent être
  coordonnés par l'application.

Les équations idéales servent à comprendre et dimensionner la rampe, pas à
garantir le profil temporel réellement émis.

## Références

- David Austin, *Generate stepper-motor speed profiles in real time*,
  EE Times, janvier 2005 :
  [article en PDF](https://nexp.pt/pdf/Stepper_Motor_Speed_Profile.pdf).
- Atmel/Microchip, *AVR446: Linear speed control of stepper motor*, notamment
  la section 2.3.1 sur le facteur `0,676` :
  [note d'application](https://ww1.microchip.com/downloads/en/appnotes/doc8017.pdf).

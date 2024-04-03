# Handleiding

In deze handleiding is te vinden hoe dit beroepsproduct gebouwd en gerund kan worden.

## Vereisten van het systeem

- Gecompileerd op Ubuntu. De versie is Ubuntu 22.04.4 LTS (jammy).
- De uitgave van ros2 op deze computer is Humble Hawksbill.


## Voorbereiding

Vanuit deze directory kun je in de folder genaamd 'rviz' een bestand vinden die 'urdf.rviz' heet.
Verder kun je in de folder genaamd 'urdf' de urdf-files vinden van de cup en de lynxmotion_arm.

In regels 40 en 152 van urdf.rviz zijn de volgende regels te vinden:

```bash
Regel 40:
Description File: /home/max/Documents/WoR/simulatie/simulatie-opdracht/src/implementation/urdf/lynxmotion_arm.urdf

Regel 152
Description File: /home/max/Documents/WoR/simulatie/simulatie-opdracht/src/implementation/urdf/cup.urdf
```

In deze twee regels moeten de bestandslocaties van lynxmotion_arm.urdf en cup.urdf op jouw computer staan (vanuit de home-directory), anders zal de robotarm als enige zichtbaar zijn in Rviz, en het bekertje niet.

Compileer daarna de code (in dezelfde directory als deze README):
```bash
colcon build
```

Uiteraard moet je bij elke nieuwe terminal dat je opent het project sourcen:
```bash
. install/setup.bash
```

## Gebruik van het programma

Als je klaar bent met de voorbereiding, kun je het programma runnen met het volgende commando:
```bash
ros2 launch implementation display.launch.py
```

#### Commando's voor de robotarm

Vervolgens kun je (op een andere terminal) ros2-messages publiceren op een topic genaamd /arm_command.

Een voorbeeld om de robotarm rechtop te zetten:
```bash
ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P1500 #1 P1300 #2 P500 #3 P1500 #5 P1500 T2000'}"
```

Het is belangrijk dat voor elke servo dat je wilt bewegen, je een stukje neerzet met een # (voor de index) en met een P (voor de pwm, moet tussen 500 en 2500 zitten).
Daarnaast moet het commando altijd met een T met een getal daarachter (het aantal milliseconden van de beweging).
Je hoeft je geen zorgen te maken over hoeveel spaties je gebruikt of hoeveel servo's je in één keer wilt bewegen.
Uiteraard kun je ook 'stop' geven om de robotarm te stoppen.

Andere voorbeelden zijn:
```bash
ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#1 P1000 T500'}"
ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#3 P1200 #4 P2300 T100'}"
ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: '#0 P2000 #1 P2200 #2 P1400 #3 P1050 #4 P1100 T5000'}"
ros2 topic pub -1 /arm_command std_msgs/msg/String "{data: 'stop'}"
```

#### Demoscript

Er is ook een demoscript (genaamd demo.sh) die een aantal vooraf ingestelde commando's uitvoert. Hierbij zie je dat de robot rechtop gaat staan, het bekertje oppakt, het bekertje laat vallen, het bekertje opnieuw oppakt, het bekertje schudt en het bekertje opnieuw laat vallen.

Open een nieuwe terminal, ga naar dezelfde directory als deze readme en doe het volgende (terwijl het hoofdprogramma al draait):
```bash
./demo.sh
```

Mocht je een 'Permission denied' krijgen, doe dan:
```bash
sudo chmod 777 demo.sh
```
en probeer het daarna opnieuw.


## Gerealiseerde eisen

### Virtuele servo controller

VS01: De virtuele controller luistert naar een topic waarop string messages in
het formaat van de SSC-32U worden geplaatst. Van de interface
moeten ten minste commando’s zijn opgenomen voor het verplaatsen van
de servo’s met een ingestelde duur en het stoppen van de servo’s.

Is gerealiseerd, het programma luistert naar dit soort commando's via het topic /arm_command.

VS02: De virtuele controller reageert op het topic (zie eis VS01) door
bijbehorende joint_state messages te publiceren.

Is gerealiseerd, bij een valide commando worden de joint states van de arm ge-broadcast.

VS03: De virtuele robotarm wordt gevisualiseerd in Rviz (een URDF-model van
de arm is beschikbaar op OnderwijsOnline).

Is gerealiseerd, de robotarm is in Rviz te zien.

VS04: De virtuele robotarm gedraagt zich realistisch m.b.t. tijdgedrag (servo’s
roteren kost tijd en gaat geleidelijk).

Is gerealiseerd, de duur van de beweging van de robotarm kan via het commando worden aangepast.

VS05: De virtuele robotarm kan op een willekeurige plaats in de virtuele wereld
geplaatst worden.

Niet gerealiseerd. De positie kan wel worden ingesteld in de launch file, maar dit gaat niet willekeurig.

### Virtueel bekertje

VC01: Er kan op een willekeurige plek in de virtuele wereld een bekertje
geplaatst worden.

Niet gerealiseerd. De positie kan wel worden ingesteld in de launch file, maar dit gaat niet willekeurig.

VC02: Publiceert een 3D-visualisatie van het bekertje voor Rviz.

Is gerealiseerd. Het bekertje is in Rviz te zien.

VC03: Detecteert de relevante punten van de gripper.

Is gerealiseerd. Het bekertje kijkt of hij opgepakt moet worden op basis van de posities van de left_gripper en de right_gripper.

VC04: Visualiseert de gedetecteerde punten van de gripper.

Niet gerealiseerd. De relevante punten van de gripper worden niet specifiek aangegeven.

VC05: Visualiseert wanneer de gripper het bekertje vastheeft.

Niet gerealiseerd. Het bekertje verandert niet van kleur o.i.d. als die wordt opgepakt.

VC06: Het bekertje beweegt mee met de gripper (als hij vastgehouden wordt).

Is gerealiseerd.

VC07: Bekertje is onderhevig aan zwaartekracht wanneer losgelaten.

Is gerealiseerd.

VC08: Bekertje bepaalt en publiceert zijn positie.

Is gerealiseerd. De positie van het bekertje wordt ge-broadcast.

VC09: Bekertje bepaalt en publiceert zijn snelheid.

Niet gerealiseerd. Het bekertje bepaalt wel zijn snelheid als die valt, maar publiceert dat niet.

VC10: Snelheid wordt getoond met rqt_plot.

Niet gerealiseerd.

### Demonstratie-infrastructuur

DI01: Een demoscript stuurt over de tijd een sequentie van commando’s naar
de armcontroller.

Is gerealiseerd. Er is een demoscript aanwezig (demo.sh) die ervoor zorgt dat de robot het bekertje kan oppakken en weer kan laten vallen.

DI02: Locatie van het bekertje wordt in de roslaunch-configuratie bepaald.

Is gerealiseerd. In display.launch.py kan de beginpositie van het bekertje worden ingesteld.

DI03: Locatie van de arm in de wereld wordt in de roslaunch-configuratie bepaald.

Is gerealiseerd. In display.launch.py kan de beginpositie van de robotarm worden ingesteld.
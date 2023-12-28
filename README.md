# CWDecoder
Morse decoder based on ESP32 and Goertzel Algorithm

 F4LAA : ESP32 Real Time Morse Decoder
 12/12/2023 : MorseDecoder V1.0
 
   Trouvé grace à la vidéo YouTube de G6EJD : https://www.youtube.com/watch?v=9OWl8zOHgls 

   Le code ESP32 est disponible ici : https://github.com/G6EJD/ESP32-Morse-Decoder

   Carte : NodeMCU-32S

   Adaptation: 
     Utilisation de l'écran TFT 4" avec la librairie rapide TFT_eSPI
     https://github.com/Bodmer/TFT_eSPI
     ATTENTION :
       La définitions des Pins n'est pas dans ce programme, 
       ==> il faut adapter le fichier E:\Users\syst4\Documents\Arduino\libraries\TFT_eSPI-master\User_Setup.h de la librairie TFT_eSPI

 24/12/2023 : Modifications V1.0 ==> V1.2 :
   - Ajout d’un encodeur rotatif permettant de modifier manuellement les paramètres de l’Algo.
   - Recherche automatique de la meilleure fréquence à mesurer par l’Algo Goertzel, qui change d’un opérateur à l’autre.
   - Réglage automatique du niveau d’entrée grâce au potentiomètre numérique MCP41010 (10k, pilotable depuis le bus SPI)
     Le volume varie beaucoup d’un opérateur à l’autre, F5NWY arrivant S9+40 chez moi 😊

 25/12/2023 : Modifications V1.2 ==> V1.3 :
   - Intégration de l'Algo de F5BU basé sur la mesure des temps de dot / dash / silences (compile mais ne fonctionne pas encore 😊)

 27/12/2023 : Modifications V1.2 ==> V1.3b :
   - remplacement compteur (cptFreqLow) par la mesure du temps (using startLowSignal)
   - Suppression du code F5BU

 27/12/2023 : Modifications V1.3b ==> V1.3c :
   - Modif Algo : On base tout sur la mesure de moyDot, puis calcul de moyDash et moySP, séparation . / - using discri
     Ca marche beaucoup moins bien ☹️
   - Ajout trace pour visualiser sur le TFT : les min/max H et L, et les moyennes (moyDot, moyDash, moySP, discri, bMoy et barGraph)

 28/12/2023 : Modifications V1.3c ==> V1.3d :
   - Retour à l'Algo de YT utilisant hightimesavg comme discriminateur . / -
   - Utilisation de VSCode + PlatformIO (à la place de l'IDE Arduino) : Le programme MorseDecoderV1.3d.ino devient src/main.cpp
   - Republication sur GitHub (sans le code de F5BU, et au format PlatformIO)


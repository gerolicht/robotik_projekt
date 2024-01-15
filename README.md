# robotik_projekt
Das Package enthält entsprechend den Übungen ein Beispiel zur Nutzung des Laserscanners sowie zur Nutzung der Kamera. 

## Laserscanner
Das Beispiel ist dem Verhalten eines einfachen Staubsaugroboter nachempfunden. Sofern sich ein Hindernis im definierten Bereich befindet, dreht sich der Roboter zur Seite. Ansonsten fährt der Roboter geradeaus. Für die Hinderniserkennung wird lediglich ein Messwert pro Laserscan ausgewertet.
```
ros2 run robotik_projekt drive_with_laserscanner
```

## Kamera
Das Beispiel konvertiert das komprimierte Kamerabild zu einem OpenCV-Graustufenbild und zeigt dieses an. In einem extra Fenster wird die unterste Bildzeile des Graustufenbildes gesondert visualisiert. Eine weitere Auswertung muss noch implementiert werden.
```
ros2 run robotik_projekt line_following
```

### Abhängigkeiten
Das Kamera-Beispiel nutzt OpenCV. Die Installation ist hier beschrieben: https://docs.opencv.org/5.x/d2/de6/tutorial_py_setup_in_ubuntu.html
```
sudo apt install python3-opencv
```

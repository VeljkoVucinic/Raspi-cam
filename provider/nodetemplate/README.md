# Template für eine ROS2-Node
Code-Template für eine Node auf Basis von ROS2 (C++) nach der Bachelorarbeit von Michel Brodatzki.

Dieser Code umfasst ein grundlegendes Lifecycle-System, dezentrale Abhängigkeitensuche, Abhängigkeitenmonitoring und eine Implementierung von einem Plug-and-Play-Konstrukt.

## Voraussetzungen an die Entwicklungsumgebung
Zur Entwicklung mit diesem Code benötigt es ein wenig vorinstallierte Software.
### ROS 2
Um mit ROS2-Code zu arbeiten, benötigt man zuerst einen kompletten ROS2-Workspace.
[Eine Anleitung, wie man diese auf Linux, macOS und Windows einrichtet, findet sich hier!](https://index.ros.org/doc/ros2/Installation/Eloquent/)

**Wichtig:** Dieser Link führt zur Installationsanleitung für ROS 2 in der Version "Eloquent Elusor", welche selbst zum Zeitpunkt der Bachelorarbeit nicht die aktuellste Version ist. Da die (derzeit) neuste Version "Foxy Fritzroy" jedoch relativ neu ist, gibt es noch kein offizielles ROS2-Docker-Image für ARM32v7-Architekturen (Raspberry Pi u.ä. - auch Raspberry Pi 4 mit aktuellem Raspbian). Solltest du diese Anleitung gerade zum ersten Mal durchlesen, bitte ich dich [kurz in die Sektion "Supported tags and respective Dockerfile links" der entsprechenden dockerhub Seite für arm32v7/ros zu schauen, ob der Tag foxy existiert](https://hub.docker.com/r/arm32v7/ros). Sollte dies der Fall sein, bitte ich dich das Tag in der ersten Zeile des Dockerfiles, sowie dieses README anzupassen.

### Entwicklungsumgebung
Im Rahmen der Installationsanleitung für Windows wird Visual Studio Community installiert. Das ist meiner Meinung nach fürs Programmieren und Debuggen voll ausreichend. Persönlich habe ich jedoch jenachdem an welchem Gerät ich gerade am Code gearbeitet habe wahlweise Visual Studio Code (bei Arbeit an einem PC mit Linux) oder neovim (bei Arbeit an einem Rechner mit Linux) verwendet. Das hat auch wunderbar funktioniert.

### Debugging
Hier kann ich nur über meinen Debuggingworkflow in Windows reden. Solltest du Linux oder macOS verwenden, ergänze doch bitte diesen Abschnitt!

ROS2 exportiert die entsprechenden Debuggingsymbols für die Plattform auf welcher kompilliert wurde. Mit Visual Studio Community kann man also einfach die .exe-Datei aus dem Projektordner öffnen und ausführen. Die Ressourcenanzeige, sowie die Debuggingfunktionen sollten direkt funktionieren. Bei manuellem Anhalten des Prozesses oder einem Crash wird auch automatisch die "schuldige" Codezeile angezeigt.

In jedem Fall kann man aber auch strategisch mit ```RCLCPP_INFO(rclcpp::get_logger("LOGGER NAME"), "Auszugebender Text", ...);``` Text in die Kommandozeilenausgabe ausgeben. Analog zu ```printf```, kann man im auszugebenden Text für weitere Infos auch sogenannte "format specifiers" benutzen, welche für die Ausgabe durch die als weitere Parameter übergebene Variablen ersetzt werden.\
Beispielsweise würde ```RCLCPP_INFO(rclcpp::get_logger("Temperaturmesser"), "Hallo %s! Wir haben aktuell %d Grad Celsius.", "Welt", 10);``` eine Ausgabe wie ```[Temperaturmesser] Hallo Welt! Wir haben aktuell 10 Grad Celsius``` erzeugen.

## Voraussetzung an die Produktivumgebung
Die genaue Methode der Orchestrierung zu evaluieren, war nicht Teil meiner Bachelorarbeit, jedoch sind wir bei [k3s (einer Art Kuberenetes light)](https://k3s.io/) gelandet, welches mit Dockercontainer läuft. Eine Anleitung, wie man k3s auf Raspberry Pis einrichtet, findet sich im README des Stammverzeichnis dieses Repositories.

An die Hardware gibt es nur die Voraussetzung, dass es die verwendete Orchestrierungsmethode unterstützt, und - um dieses Template in der aktuellen Form voll benutzen zu können - eine Architektur hat, unter welche ROS2 läuft bzw. für die es ein ROS2-Image gibt.

ROS2-Images für die von mir verwendeten Architekturen sind

| Architektur |   Image   | Von mir verwendeter Tag |
|-------------|-----------| --------- |
| ARM32v7 (Raspberry Pi 4 o.ä.) | arm32v7/ros | eloquent |
| x86 (handelsüblicher Rechner) | osrf/ros | eloquent-desktop (foxy-desktop existiert jedoch) |


## Arbeiten mit dem Template
In diesem Template gibt es einige Stellen, die von zukünftigen Programmierer:innen ergänzt und verändert werden können, sollten oder müssen. In dieser Liste sind die meisten (bzw. hoffentlich alle) dieser Stellen erklärt. Allgemein gilt aber, dass solche Stellen im Code durch Kommentare markiert sind, welche mit ```//!@``` beginnen. Diese Kommentare können beispielsweise mit dem Visual Studio Code-Plugin "Better Comments" hervorgehoben werden, oder man kann in der Entwicklungsumgebung (sollte diese das unterstützen) alle Dateien nach ```//!@``` durchsuchen.

### Metadateien
**package.xml**\
Diese Datei beinhaltet Informationen zur aktuellen Package. Hier muss im ```name```-Tag der Paketname verändert werden. Weiterhin kann hier der Maintainer - samt E-Mail - hinterlegt werden.

**CMakeLists.txt**\
Diese Datei beinhaltet Anweisungen für CMake. Hier muss im ```project```-Tag der Paketname eingefügt werden. Auch muss weiter unten der Executablename überall angepasst werden.

Wichtig: Sollten neue Quellcodedateien zum Projekt hinzugefügt werden, muss der Pfad im ```add_executable```-Tag angehängt werden.

### Allgemeine Einstellungen
**settings.cpp**\
In dieser Datei kann man folgende Parameter einstellen:
* Nodename
* Maximale Anzahl an verpasster Heartbeats, bis eine Abhängigkeit als "verloren" gilt
* Abhängigkeiten
* Eigene Capabilities mit Fehleroffset
* Metadaten der Node

### Lifecycle
**lifecycle_node.cpp**\
In dieser Datei kann man das Lifecycle-Update-Interval anpassen. Standardmäßig ist dieses bei 500ms. Dieses Interval beeinflusst normalerweise keine Publisher, Subscriber, Services oder Actions, sondern nur wie regelmäßig die Update-Methode von States aufgerufen wird.

**Neuen State hinzufügen**
1. Einen existierenden State kopieren (sowohl die .cpp, als auch die .hpp-Datei)
2. Sichergehen, dass OnBegin ```true``` zurückgibt
3. Daran denken einzustellen, ob die Node in diesem State aktiv ist oder nicht
4. Den Namen des neuen States von ::GetName() zurückgeben lassen
5. In den Headerdateien der States, die in den neuen State wechseln können die ```allowed_transitions_```-Map ergänzen
6. Die Quelldatei in der CMakeLists.txt zum ```add_executable```-Tag hinzufügen

### Capabilities
Capabilities habe ich gehandhabt, indem ich eigene Klassen im Singleton-Entwurfsmuster geschrieben haben, die sich ausschließlich mit einer Capability befassen. Ein Beispiel findet sich im Demonstrationsprojekt mit der PiCamera.

Man darf aber nicht vergessen, dass die neue Capability in der ```settings.cpp``` eingetragen werden muss.

### Plug and Play
**node_network_manager.cpp**\
Diese Datei ist Dreh- und Angelpunkt des Plug and Play-Systems.

In der Methode ```NodeNetworkManager::pnp_should_swap_dependency``` wird entschieden, ob eine neue Abhängigkeit über der Aktuellen bevorzugt wird. Hier muss ein Vergleich implementiert werden.

```NodeNetworkManager::pnp_discovery_subscriber_function``` reagiert unter anderem auf neugefundene Abhängigkeiten. Hier kann Verhalten für den Fall einer neuen Abhängigkeit hinzugefügt werden. Beispielsweise können hier Topics der neuen Node abonniert werden.

Der meiner Meinung nach wichtigste Bereich befindet sich in ```NodeNetworkManager::InitiateEndpoints```, wo die ROS2-Endpoints, wie Publisher, Subscriber, Services, Actions usw. eingerichtet werden. Diese sind fundamentaler Bestandteil der Kommunikation zwischen Nodes in dieser Implementation.

**settings.cpp**\
Wie zuvor erwähnt, werden in dieser Datei Abhängigkeiten und die eigenen Capabilities beschrieben. Diese Informationen werden vom NodeNetworkManager verwendet, um sich selbst im Netzwerk zu announcen und um neuen Nodes zu finden, welche die gewünschten Abhängigkeiten erfüllen können.

## Kompilieren und Starten
Um die Node dann kompilieren und starten zu können, habe ich Skripts dazugelegt.

Mit ```build_container.sh``` wird aus dem Quellcode ein Container erstellt. Dieses Skript muss auf der Zielarchitektur ausgeführt werden.\
Der Name des erstellten Containers ist ```ros2_{PACKAGE NAME}```.

Um mit Orchestratoren auf diese Container zugreifen zu können, empfehle ich (und so habe ich es beim Testen auch gemacht) im Netzwerk einen privaten Docker registry Server auszuführen. [Eine Anleitung wie dies geht, findet sich hier!](https://docs.docker.com/registry/deploying/)\
In meinem Fall konnte ich mit k3s-Deployments dann diese Images auf die verschiedenen Raspberry Pis verteilen. [Eine Anleitung, wie man k3s mit privaten Registries verbindet, findet sich hier!](https://rancher.com/docs/k3s/latest/en/installation/private-registry/)

Mit ```run_container.sh``` kann der soeben erstellte Container ausgeführt werden.

## Demonstrationsprojekt
Mit diesem Template habe ich eine Demonstration entwickelt. Diese findet sich im Repository im Verzeichnis "ba-brodatzki_demo" und beinhaltet 3 Nodes, welche die Funktionen dieses Templates grundlegend zeigen.

Eine Anleitung was genau diese Demonstration macht, wie sie funktioniert, wie man sie aufbaut und wie das Resultat aussieht, findet sich im eben genannten Verzeichnis.
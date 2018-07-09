# <img src="logo/ontologenius.png" width="150"> ontolo**G**enius
This repository is a ROS package to link, explore and interrogate ontologies.

***

Matser : [![Build Status](https://travis-ci.org/sarthou/ontologenius.svg?branch=master)](https://travis-ci.org/sarthou/ontologenius)

Dev : [![Build Status](https://travis-ci.org/sarthou/ontologenius.svg?branch=dev)](https://travis-ci.org/sarthou/ontologenius)

***

[**Wiki homepage**](https://github.com/sarthou/ontologenius/wiki#-ontologenius)

[**1. Installation**](https://github.com/sarthou/ontologenius/wiki/Installation#installation)

[**2. Create ontology**](https://github.com/sarthou/ontologenius/wiki/Create-ontology)

[**3. Launch ontoloGenius**](https://github.com/sarthou/ontologenius/wiki/Launch-ontoloGenius)

[**4. Load ontology**](https://github.com/sarthou/ontologenius/wiki/Load-ontology)

[**5. Exploration**](https://github.com/sarthou/ontologenius/wiki/Exploration)
 - [**5.1. Classes exploration**](https://github.com/sarthou/ontologenius/wiki/Classes-exploration)
 - [**5.2. Properties exploration**](https://github.com/sarthou/ontologenius/wiki/Properties-exploration)
 - [**5.3. Individuals exploration**](https://github.com/sarthou/ontologenius/wiki/Individuals-exploration)

[**6. Reason**](https://github.com/sarthou/ontologenius/wiki/Reason)

[**7. Programming**](https://github.com/sarthou/ontologenius/wiki/Programming)

[**8. OntoloGUI**](https://github.com/sarthou/ontologenius/wiki/ontoloGUI)

***

## Run the package

A *.launch* file is available to run the package:
```sh
$ roslaunch ontologenius ontologenius.launch
```

On this launcher file, you will find an argument indicating the path to the ontology files.
> The default ontologies are very simple and I encourage you to contribute to their development!

You can add your own ontology files in the argument if you want them to be loaded at the beginning of the program.

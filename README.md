# <img src="logo/ontologenius.png" width="150"> ontolo**G**enius
This repository is a ROS package to link, explore and interrogate ontologies.

[**Wiki homepage**](https://github.com/sarthou/ontologenius/wiki#-ontologenius)

[**1. Installation**](https://github.com/sarthou/ontologenius/wiki/Installation#installation)

[**2. Create ontology**](https://github.com/sarthou/ontologenius/wiki/Create-ontology)

[**3. Load ontology**](https://github.com/sarthou/ontologenius/wiki/Load-ontology)

[**4. Exploration**](https://github.com/sarthou/ontologenius/wiki/Exploration)

***

## Run the package

A *.launch* file is available to run the package:
```sh
$ roslaunch ontologenius ontologenius.launch
```

On this launcher file, you will find an argument indicating the path to the ontology files.
> The default ontologies are very simple and I encourage you to contribute to their development!

You can add your own ontology files in the argument if you want them to be loaded at the beginning of the program.

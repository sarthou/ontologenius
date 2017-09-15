# <img src="logo/ontologenius.png" width="150"> ontolo**G**enius
This repository is a ROS package to link, explore and interrogate ontologies.

## Installation

To install this, put in your catkin workspace:
```
$ cd ~/catkin_ws/src/

$ git clone https://github.com/sarthou/ontologenius.git

$ cd ..

$ catkin_make
```

You will need the requests package. You can install it in the catkin_ws:
```
$ git clone git://github.com/kennethreitz/requests.git
$ export PYTHONPATH=~/catkin_ws/devel/lib/python2.7/site-packages:$PYTHONPATH
$ cd requests
$ python setup.py install --prefix=~/catkin_ws/devel
```
> you might have to create ~/catkin_ws/devel/lib/python2.7/site-packages
```
$ mkdir ~/catkin_ws/devel/lib/python2.7/site-packages
```

## Run the package

A *.launch* file is available to run the package:
```
$ roslaunch ontologenius ontologenius.launch
```

On this launcher file, you will find an argument indicating the path to the ontology files.
> The default ontologies are very simple and I encourage you to contribute to their development!

You can add your own ontology files in the argument if you want them to be loaded at the beginning of the program.

# Load ontologies

The ontologenius allows to load several ontologies throw files on your computer (at the launch of the program) or from internet.
This node is able to load different ontologies and link them automatically.

- Add the ontologies from internet:
```
$ rosservice call /ontoloGenius/actions "{param: 'https://raw.githubusercontent.com/sarthou/ontologenius/master/files/attribute.owl', action: 'add' }"
```

- When you have loaded all the desired ontologies, you must close it to link all the entities:
```
$ rosservice call /ontoloGenius/actions "{param: '', action: 'close'}"
```

- If you want to erase everything you have previously loaded, call:
```
$ rosservice call /ontoloGenius/actions "{param: '', action: 'reset'}"
```
> The action will erase ALL ontologies, even those loaded at the launch of the program

# Usage

### Exploration
Ontologenius allow a bi-directonnal exploration.

- ***getDown*** will give you all classes below that given in the parameter
```
$ rosservice call /ontoloGenius/actions "{param: 'human', action: 'getDown'}"
```
> result => child human man woman

- ***getUp*** will give you all classes above that given in the parameter
```
$ rosservice call /ontoloGenius/actions "{param: 'human', action: 'getUp'}"
```
> result => activity agent alive animate attribute entity human vitality

### Relationship test
Ontologenius makes it possible to test the relationship between classes

#### Syntax

Symbol          | Meaning
-------------   | -------------
_               | AND
|               | OR
!               | NOT (not implemented yet)
=               | equality test

> Do not use _ in your class and individual names

#### Create questions

- The main subject is on the left:
If we write: *man=entity*, this means that we want to know if a man is an entity.
This is different to write: *entity=man* because here we ask if an entity is a *man* !!
> You will tell me that an entity can be a man but we can not be sure ...

- You can combine classes to create new ones on your query with AND
If we write: *man=entity_alive*, we ask if a man is an entity that is also alive.
The class combination is also possible on the subject side: *old_man=entity*

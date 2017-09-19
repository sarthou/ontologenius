# <img src="logo/ontologenius.png" width="150"> ontolo**G**enius
This repository is a ROS package to link, explore and interrogate ontologies.

## Installation

To install this, put in your catkin workspace:
```sh
$ cd ~/catkin_ws/src/

$ git clone https://github.com/sarthou/ontologenius.git

$ cd ..

$ catkin_make
```

You will need the requests package. You can install it in the catkin_ws:
```sh
$ git clone git://github.com/kennethreitz/requests.git
$ export PYTHONPATH=~/catkin_ws/devel/lib/python2.7/site-packages:$PYTHONPATH
$ cd requests
$ python setup.py install --prefix=~/catkin_ws/devel
```
> you might have to create ~/catkin_ws/devel/lib/python2.7/site-packages

```sh
$ mkdir ~/catkin_ws/devel/lib/python2.7/site-packages
```

## Run the package

A *.launch* file is available to run the package:
```sh
$ roslaunch ontologenius ontologenius.launch
```

On this launcher file, you will find an argument indicating the path to the ontology files.
> The default ontologies are very simple and I encourage you to contribute to their development!

You can add your own ontology files in the argument if you want them to be loaded at the beginning of the program.

## Load ontologies

The ontologenius allows to load several ontologies through files on your computer (at the launch of the program) or from internet.
This node is able to load different ontologies and link them automatically.

- Add the ontologies from internet:
```sh
$ rosservice call /ontoloGenius/actions "{param: 'https://raw.githubusercontent.com/sarthou/ontologenius/master/files/attribute.owl', action: 'add' }"
```

- When you have loaded all the desired ontologies, you must close it to link all the entities:
```sh
$ rosservice call /ontoloGenius/actions "{param: '', action: 'close'}"
```

- If you want to erase everything you have previously loaded, call:
```sh
$ rosservice call /ontoloGenius/actions "{param: '', action: 'reset'}"
```
> The action will erase ALL ontologies, even those loaded at the launch of the program

## Usage

### Exploration
Ontologenius allow a bi-directonnal exploration.

- ***getDown*** will give you all classes below the one given in the parameter
```sh
$ rosservice call /ontoloGenius/actions "{param: 'human', action: 'getDown'}"
```
> result => child human man woman

- ***getUp*** will give you all classes above the one given in the parameter
```sh
$ rosservice call /ontoloGenius/actions "{param: 'human', action: 'getUp'}"
```
> result => activity agent alive animate attribute entity human vitality

- ***getDisjoint*** will give you all the disjoint classes of the one given in the parameter
```sh
rosservice call /ontoloGenius/actions "{param: 'human', action: 'getDisjoint'}"
```
> result => animal robot

### Relationship test
Ontologenius makes it possible to test the relationship between classes

#### Syntax

Symbol          | Meaning
-------------   | -------------
_               | AND
\|              | OR
!               | NOT
=               | equality test
!               | NOT
\-              | comment

> Do not use _ in your class and individual names

#### Create questions

- The main subject is on the left:  
If we write: ***man=entity***, this means that we want to know if a *man* is an *entity*.  
This is different to write: ***entity=man*** because here we ask if an *entity* is a *man* !!  
> You will tell me that an entity can be a man but we can not be sure ...

- Combine classes to create new ones on your query with AND  
If we write: ***man=entity_alive***, we ask if a man is an entity that is **also** alive.  
The class combination is also possible on the subject side: ***old_man=entity***  

- Test multiple possibilities with OR  
If we write: ***desk=affair|furniture***, we ask if desk is an affair OR a furniture.  
It is also possible to use it on the side of the subject: ***desk|book=furniture***  

- Test the opposite with NOT  
If we write: ***man=!robot***, we ask if a man is NOT a robot.  
This function exploits the disjoint description. So in this example, test ***!robot*** is equivalent to testing ***human|animal***.  
> /:exclamation:\\ The Not symbol is not taken into account on the subject side

- Comment on your courses
We often use identifiers to distinguish names that do not represent the same entity. For example, if you have two red cubes, you will call them red_cube_1 and red_cube_2.
To avoid this problem and do not create any problems with identifiers that do not represent anything in our representation of the world, you can put these ids in comment with **'-'**.
If we write: ***red_cube-1 = object***, we ask if a red cube is an object but without taking into account the identifier.
Comments can be used on both sides of the equality test and after each class name.
> You can put everything in the comment: robot-theTinyOne = agent

- Combine all symbols:  
Now, you can create complex queries by combining the different symbols: ***red_cube|young_animal=color_!animal|color_object***  
> This example is strange but you see the meaning ...  

#### Send a question

To send a question, just put it in the stock service like this:  
```sh
$ rosservice call /ontoloGenius/actions "{param: 'blue_box|red_cube=alive', action: 'test'}"
```
The service will return the answer as *true* or *false*.

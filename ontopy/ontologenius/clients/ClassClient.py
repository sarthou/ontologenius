from .OntologyClient import OntologyClient

class ClassClient(OntologyClient):
    """The ClassClient class provides an abstraction of ontologenius classes ROS service.
       The ontologenius classes service allows the exploration of classes contained by ontologenius core.
       This class is based on ClientBase and so ensure a persistent connection with ontologenius/class service.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs a class client..
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name(str) should be defined as ''.
        """
        if name == '':
            OntologyClient.__init__(self, "class")
        else:
            OntologyClient.__init__(self, "class/" + name)

    def getDown(self, name, depth = -1):
        """Gives all classes (str[]) below the one given in the parameter: name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDown", param)

    def getDisjoint(self, name):
        """Gives all the disjoint classes (str[]) of the one given in the parameter: name(str)."""
        return self.call("getDisjoint", name)

    def getOn(self, name, property, selector = ''):
        """Gives all the classes (str[]) pointed by the property property(str) and applied to the class name(str).
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) class.
           The default value '' represents no restriction on the result.
        """
        param = name + ":" + property
        if selector != '':
            param += " -s " + selector
        return self.call("getOn", param)

    def getFrom(self, property, name, selector = ''):
        """Gives all the classes (str[]) having the given property property(str) and pointing to the class name(str).
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) class.
           The default value '' represents no restriction on the result.
        """
        param = name + ":" + property
        if selector != '':
            param += " -s " + selector
        return self.call("getFrom", param)

    def getWith(self, class_from, class_to, selector = '', depth = -1):
        """Gives all the properties (str[]) linking the two classes class_from(str) and class_to(str).
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) property.
           The default value '' represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = class_from + ":" + class_to
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getWith", param)

    def getRelatedFrom(self, property):
        """Gives all the classes (str[]) possessing the property property(str)."""
        return self.call("getRelatedFrom", property)

    def getRelatedOn(self, property):
        """Gives all the classes (str[]) pointed to by the property property(str)."""
        return self.call("getRelatedOn", property)

    def getRelatedWith(self, name):
        """Gives all the classes (str[]) having a property pointing to the class name(str)."""
        return self.call("getRelatedWith", name)

    def getRelationFrom(self, name, depth = -1):
        """Gives all the properties (str[]) applied to the class name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRelationFrom", param)

    def getRelationOn(self, name, depth = -1):
        """Gives all the properties (str[]) going to the class name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRelationOn", param)

    def getRelationWith(self, name):
        """Gives all the classes (str[]) pointed by a property applied to the class name(str)."""
        return self.call("getRelationWith", name)

    def getDomainOf(self, name, selector = '', depth = -1):
        """Gives all the properties (str[]) for which the class name(str) is part of the domain.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) property.
           The default value '' represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the class to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDomainOf", param)

    def getRangeOf(self, name, selector = '', depth = -1):
        """Gives all the properties (str[]) for which the class name(str) is part of the range.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) property.
           The default value '' represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the class to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRangeOf", param)

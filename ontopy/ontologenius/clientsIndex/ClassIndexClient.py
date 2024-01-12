from .OntologyIndexClient import OntologyIndexClient

class ClassIndexClient(OntologyIndexClient):
    """The ClassIndexClient class provides an abstraction of ontologenius classes ROS service.
       The ontologenius classes service allows the exploration of classes contained by ontologenius core based on indexes.
       This class is based on ClientBaseIndex and so ensure a persistent connection with ontologenius/class_index service.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs a class client..
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name(str) should be defined as ''.
        """
        if name == '':
            OntologyIndexClient.__init__(self, "class_index")
        else:
            OntologyIndexClient.__init__(self, "class_index/" + name)

    def getDown(self, index, depth = -1):
        """Gives all classes (integer[]) below the one given in the parameter: index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getDown", param)

    def getDisjoint(self, index):
        """Gives all the disjoint classes (integer[]) of the one given in the parameter: index(integer)."""
        return self.callIndexes("getDisjoint", str(index))

    def getOn(self, index, property, selector = 0):
        """Gives all the classes (integer[]) pointed by the property property(integer) and applied to the class index(integer).
           The optional selector(integer) parameter can be set to only get results inheriting from the selector(integer) class.
           The default value 0 represents no restriction on the result.
        """
        param = str(index) + ":" + str(property)
        if selector != 0:
            param += " -s " + str(selector)
        return self.callIndexes("getOn", param)

    def getFrom(self, property, index, selector = 0):
        """Gives all the classes (integer[]) having the given property property(integer) and pointing to the class index(integer).
           The optional selector(integer) parameter can be set to only get results inheriting from the selector(integer) class.
           The default value 0 represents no restriction on the result.
        """
        param = str(index) + ":" + str(property)
        if selector != 0:
            param += " -s " + str(selector)
        return self.callIndexes("getFrom", param)

    def getWith(self, class_from, class_to, selector = 0, depth = -1):
        """Gives all the properties (integer[]) linking the two classes class_from(integer) and class_to(integer).
           The optional selector(integer) parameter can be set to only get results inheriting from the selector(integer) property.
           The default value 0 represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(class_from) + ":" + str(class_to)
        if selector != 0:
            param += " -s " + str(selector)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getWith", param)

    def getRelatedFrom(self, property):
        """Gives all the classes (integer[]) possessing the property property(integer)."""
        return self.callIndexes("getRelatedFrom", str(property))

    def getRelatedOn(self, property):
        """Gives all the classes (integer[]) pointed to by the property property(integer)."""
        return self.callIndexes("getRelatedOn", str(property))

    def getRelatedWith(self, index):
        """Gives all the classes (integer[]) having a property pointing to the class index(integer)."""
        return self.callIndexes("getRelatedWith", str(index))

    def getRelationFrom(self, index, depth = -1):
        """Gives all the properties (integer[]) applied to the class index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRelationFrom", param)

    def getRelationOn(self, index, depth = -1):
        """Gives all the properties (integer[]) going to the class index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRelationOn", param)

    def getRelationWith(self, index):
        """Gives all the classes (integer[]) pointed by a property applied to the class index(integer)."""
        return self.callIndexes("getRelationWith", str(index))

    def getDomainOf(self, index, selector = 0, depth = -1):
        """Gives all the properties (integer[]) for which the class index(integer) is part of the domain.
           The optional selector(integer) parameter can be set to only get results inheriting from the selector(integer) property.
           The default value 0 represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the class to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if selector != 0:
            param += " -s " + str(selector)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getDomainOf", param)

    def getRangeOf(self, index, selector = 0, depth = -1):
        """Gives all the properties (integer[]) for which the class index(integer) is part of the range.
           The optional selector(integer) parameter can be set to only get results inheriting from the selector(integer) property.
           The default value 0 represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the class to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if selector != 0:
            param += " -s " + str(selector)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRangeOf", param)

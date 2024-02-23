from .OntologyIndexClient import OntologyIndexClient

class ObjectPropertyIndexClient(OntologyIndexClient):
    """The ObjectPropertyIndexClient class provides an abstraction of ontologenius object properties ROS service.
       The ontologenius object properties service allows the exploration of object
       properties contained by ontologenius core based on indexes.
       This class is based on ClientBaseIndex and so ensure a persistent connection with
       ontologenius/object_property_index service. The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs a class client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name should be defined as ''.
        """
        if name == '':
            OntologyIndexClient.__init__(self, "object_property_index")
        else:
            OntologyIndexClient.__init__(self, "object_property_index/" + name)

    def getDown(self, index, depth = -1):
        """Gives all properties (integer[]) below the one given in the parameter: index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getDown", param)

    def getDisjoint(self, index):
        """Gives all the disjoint properties (integer[]) of the property index(integer)."""
        return self.callIndexes("getDisjoint", str(index))

    def getDomain(self, index, depth = -1):
        """Gives all the domain classes (integer[]) of the property index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation while the value 0 corresponds to the direct domains."""
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getDomain", param)

    def getRange(self, index, depth = -1):
        """Gives all the ranges classes (integer[]) of the property index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation while the value 0 corresponds to the direct ranges."""
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRange", param)

    def getInverse(self, index):
        """Gives all the inverses properties (integer[]) of the property index(integer)."""
        return self.callIndexes("getInverse", str(index))

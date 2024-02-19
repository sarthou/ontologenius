from .OntologyClient import OntologyClient

class DataPropertyClient(OntologyClient):
    """The DataPropertyClient class provides an abstraction of ontologenius data properties ROS service.
       The ontologenius data properties service allows the exploration of data properties contained by ontologenius core.
       This class is based on ClientBase and so ensure a persistent connection with ontologenius/data_property service.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs a class client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name(str) should be defined as ''.
        """
        if name == '':
            OntologyClient.__init__(self, "data_property")
        else:
            OntologyClient.__init__(self, "data_property/" + name)

    def getDown(self, name, depth = -1):
        """Gives all properties (str[]) below the one given in the parameter: name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDown", param)

    def getDisjoint(self, name):
        """Gives all the disjoint properties (str[]) of the property name (str)."""
        return self.call("getDisjoint", name)

    def getDomain(self, name, depth = -1):
        """Gives all the domain classes (str[]) of the property name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation while the value 0 corresponds to the direct domains."""
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDomain", param)

    def getRange(self, name):
        """Gives all the ranges types (str[]) of the property name(str)."""
        return self.call("getRange", name)

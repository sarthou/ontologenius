from .OntologyIndexClient import OntologyIndexClient

class IndividualIndexClient(OntologyIndexClient):
    """The IndividualIndexClient class provides an abstraction of ontologenius individuals ROS service.
       The ontologenius individuals' service allows the exploration of individuals contained by ontologenius core based on indexes.
       This class is based on ClientBaseIndex and so ensure a persistent connection with ontologenius/individual_index service.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs an class client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name should be defined as "".
        """
        if name == '':
            OntologyIndexClient.__init__(self, "individual_index")
        else:
            OntologyIndexClient.__init__(self, "individual_index/" + name)

    def getOn(self, index, property, selector = 0):
        """Gives all the individuals (integer[]) pointed by the property(integer) and applied to the individual index(integer).
           The optional selector(integer) parameter can be set to only get results inheriting from the selector class.
           The default value 0 represents no restriction on the result.
        """
        param = str(index) + ":" + str(property)
        if selector != '':
            param += " -s " + str(selector)
        return self.callIndexes("getOn", param)

    def getFrom(self, property, index, selector = 0):
        """Gives all the individuals (integer[]) having the given property(integer) and pointing to the individual index(integer).
           The optional selector(integer) parameter can be set to only get results inheriting from the selector class.
           The default value 0 represents no restriction on the result.
        """
        param = str(index) + ":" + str(property)
        if selector != '':
            param += " -s " + str(selector)
        return self.callIndexes("getFrom", param)

    def getWith(self, indiv_from, indiv_to, selector = 0, depth = -1):
        """Gives all the properties (integer[]) linking the individual indiv_from(integer) to the individual indiv_to(integer).
           The optional selector(integer) parameter can be set to only get results inheriting from the selector property.
           The default value 0 represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(indiv_from) + ":" + str(indiv_to)
        if selector != '':
            param += " -s " + str(selector)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getWith", param)

    def getRelatedFrom(self, property):
        """Gives all the individuals (integer[]) possessing the property(integer)."""
        return self.callIndexes("getRelatedFrom", str(property))

    def getRelatedOn(self, property):
        """Gives all the individuals (integer[]) pointed to by the property(integer)."""
        return self.callIndexes("getRelatedOn", str(property))

    def getRelatedWith(self, index):
        """Gives all the individuals (integer[]) having a property pointing to the individual index(integer)."""
        return self.callIndexes("getRelatedWith", str(index))

    def getRelationFrom(self, index, depth = -1):
        """Gives all the properties (integer[]) applied to the individual index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRelationFrom", param)

    def getRelationOn(self, index, depth = -1):
        """Gives all the properties (integer[]) going to the individual index(integer).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRelationOn", param)

    def getRelationWith(self, index):
        """Gives all the individuals (integer[]) pointed by a property applied to the individual index(integer)."""
        return self.callIndexes("getRelationWith", str(index))

    def getDomainOf(self, index, selector = 0, depth = -1):
        """Gives all the properties (integer[]) for which the individual index(integer) is part of the domain.
           The optional selector(integer) parameter can be set to only get results inheriting from the selector property.
           The default value 0 represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the individual to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if selector != '':
            param += " -s " + str(selector)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getDomainOf", param)

    def getRangeOf(self, index, selector = 0, depth = -1):
        """Gives all the properties (integer[]) for which the individual index(integer) is part of the range.
           The optional selector(integer) parameter can be set to only get results inheriting from the selector property.
           The default value 0 represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the individual to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = str(index)
        if selector != '':
            param += " -s " + str(selector)
        if depth >= 0:
            param += " -d " + str(depth)
        return self.callIndexes("getRangeOf", param)

    def getType(self, index):
        """Gives all the individuals (integer[]) of the type of the given class index(integer)."""
        return self.callIndexes("getType", str(index))

    def getSame(self, index):
        """Gives all the individuals (integer[]) that are defined as being identical to the individual index(integer)."""
        return self.callIndexes("getSame", str(index))

    def getDistincts(self, index):
        """Gives all the defined individuals (integer[]) as being distinct from the individual index(integer)."""
        return self.callIndexes("getDistincts", str(index))

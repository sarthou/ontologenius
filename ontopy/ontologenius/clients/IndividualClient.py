from .OntologyClient import OntologyClient

class IndividualClient(OntologyClient):
    """The IndividualClient class provides an abstraction of ontologenius individuals ROS service.
       The ontologenius individuals' service allows the exploration of individuals contained by ontologenius core.
       This class is based on ClientBase and so ensure a persistent connection with ontologenius/individual service.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs an class client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name should be defined as "".
        """
        if name == '':
            OntologyClient.__init__(self, "individual")
        else:
            OntologyClient.__init__(self, "individual/" + name)

    def getOn(self, name, property, selector = ''):
        """Gives all the individuals (str[]) pointed by the property property(str) and applied to the individual name(str).
           The optional selector(str) parameter can be set to only get results inheriting from the selector class.
           The default value '' represents no restriction on the result.
        """
        param = name + ":" + property
        if selector != '':
            param += " -s " + selector
        return self.call("getOn", param)

    def getFrom(self, property, name, selector = ''):
        """Gives all the individuals (str[]) having the given property property(str) and pointing to the individual name(str).
           The optional selector(str) parameter can be set to only get results inheriting from the selector class.
           The default value '' represents no restriction on the result.
        """
        param = name + ":" + property
        if selector != '':
            param += " -s " + selector
        return self.call("getFrom", param)

    def getWith(self, indiv_from, indiv_to, selector = '', depth = -1):
        """Gives all the properties (str[]) linking the individual indiv_from(str) to the individual indiv_to(str).
           The optional selector(str) parameter can be set to only get results inheriting from the selector property.
           The default value '' represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = indiv_from + ":" + indiv_to
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getWith", param)

    def getRelatedFrom(self, property):
        """Gives all the individuals (str[]) possessing the property property(str)."""
        return self.call("getRelatedFrom", property)

    def getRelatedOn(self, property):
        """Gives all the individuals (str[]) pointed to by the property property(str)."""
        return self.call("getRelatedOn", property)

    def getRelatedWith(self, name):
        """Gives all the individuals (str[]) having a property pointing to the individual name(str)."""
        return self.call("getRelatedWith", name)

    def getRelationFrom(self, name, depth = -1):
        """Gives all the properties (str[]) applied to the individual name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRelationFrom", param)

    def getRelationOn(self, name, depth = -1):
        """Gives all the properties (str[]) going to the individual name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRelationOn", param)

    def getRelationWith(self, name):
        """Gives all the individuals (str[]) pointed by a property applied to the individual name(str)."""
        return self.call("getRelationWith", name)

    def getDomainOf(self, name, selector = '', depth = -1):
        """Gives all the properties (str[]) for which the individual name(str) is part of the domain.
           The optional selector(str) parameter can be set to only get results inheriting from the selector property.
           The default value '' represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the individual to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDomainOf", param)

    def getRangeOf(self, name, selector = '', depth = -1):
        """Gives all the properties (str[]) for which the individual name(str) is part of the range.
           The optional selector(str) parameter can be set to only get results inheriting from the selector property.
           The default value '' represents no restriction on the result.
           The optional depth(int) parameter can be set to limit tree propagation of the individual to a specific value.
           The default value -1 represents no propagation limitation.
        """
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRangeOf", param)

    def getType(self, name):
        """Gives all the individuals (str[]) of the type of the given class name(str)."""
        return self.call("getType", name)

    def getSame(self, name):
        """Gives all the individuals (str[]) that are defined as being identical to the individual name(str)."""
        return self.call("getSame", name)

    def getDistincts(self, name):
        """Gives all the defined individuals (str[]) as being distinct from the individual name(str)."""
        return self.call("getDistincts", name)

import rospy

from .OntologyClient import OntologyClient

class IndividualClient(OntologyClient):

    def __init__(self, name):
        if name == '':
            OntologyClient.__init__(self, "individual")
        else:
            OntologyClient.__init__(self, "individual/" + name)

    def getOn(self, name, property, selector = ''):
        param = name + ":" + property
        if selector != '':
            param += " -s " + selector
        return self.call("getOn", param)

    def getFrom(self, property, name, selector = ''):
        param = name + ":" + property
        if selector != '':
            param += " -s " + selector
        return self.call("getFrom", param)

    def getWith(self, indiv_from, indiv_to, selector = '', depth = -1):
        param = indiv_from + ":" + indiv_to
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getWith", param)

    def getRelatedFrom(self, property):
        return self.call("getRelatedFrom", property)

    def getRelatedOn(self, property):
        return self.call("getRelatedOn", property)

    def getRelatedWith(self, name):
        return self.call("getRelatedWith", name)

    def getRelationFrom(self, name, depth = -1):
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRelationFrom", param)

    def getRelationOn(self, name, depth = -1):
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRelationOn", param)

    def getRelationWith(self, name):
        return self.call("getRelationWith", name)

    def getDomainOf(self, name, selector = '', depth = -1):
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDomainOf", param)

    def getRangeOf(self, name, selector = '', depth = -1):
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getRangeOf", param)

    def getType(self, name):
        return self.call("getType", name)

    def getSame(self, name):
        return self.call("getSame", name)

    def getDistincts(self, name):
        return self.call("getDistincts", name)

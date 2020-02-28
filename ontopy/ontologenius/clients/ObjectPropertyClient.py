import rospy

from .OntologyClient import OntologyClient

class ObjectPropertyClient(OntologyClient):

    def __init__(self, name):
        if name == '':
            OntologyClient.__init__(self, "object_property")
        else:
            OntologyClient.__init__(self, "object_property/" + name)

    def getDown(self, name, depth = -1):
        param = name
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getDown", param)

    def getDisjoint(self, name):
        return self.call("getDisjoint", name)

    def getDomain(self, name):
        return self.call("getDomain", name)

    def getRange(self, name):
        return self.call("getRange", name)

    def getInverse(self, name):
        return self.call("getInverse", name)

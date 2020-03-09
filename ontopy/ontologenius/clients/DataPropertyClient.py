import rospy

from .OntologyClient import OntologyClient

class DataPropertyClient(OntologyClient):

    def __init__(self, name):
        if name == '':
            OntologyClient.__init__(self, "data_property")
        else:
            OntologyClient.__init__(self, "data_property/" + name)

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

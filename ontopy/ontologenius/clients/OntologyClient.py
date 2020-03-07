import rospy

from .ClientBase import ClientBase

class OntologyClient(ClientBase):

    def __init__(self, name):
        ClientBase.__init__(self, name)

    def getUp(self, name, depth = -1, selector = ''):
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getUp", param)

    def isA(self, name, base_class):
        res = self.getUp(name, selector = base_class)
        if len(res) == 0:
            return False
        else:
            return True

    def getName(self, name, take_id = True):
        param = name
        if take_id == False:
            param += " -i false"
        return self.callStr("getName", param)

    def getNames(self, name):
        return self.call("getNames", name)

    def getEveryNames(self, name):
        return self.call("getEveryNames", name)

    def find(self, name, take_id = True):
        param = name
        if take_id == False:
            param += " -i false"
        return self.call("find", param)

    def findSub(self, name, take_id = True):
        param = name
        if take_id == False:
            param += " -i false"
        return self.call("findSub", param)

    def findRegex(self, name, take_id = True):
        param = name
        if take_id == False:
            param += " -i false"
        return self.call("findRegex", param)

    def findFuzzy(self, name, threshold = 0.5, take_id = True):
        param = name + ' -t ' + str(threshold);
        if take_id == False:
            param += " -i false"
        return self.call("findFuzzy", param)

    def exist(self, name):
        return self.callStr("exist", name) != ''

import rospy

from .ClientBase import ClientBase

class ReasonerClient(ClientBase):

    def __init__(self, name):
        if name == '':
            ClientBase.__init__(self, "reasoner")
        else:
            ClientBase.__init__(self, "reasoner/" + name)

    def list(self):
        return self.call("list", "")

    def activeList(self):
        return self.call("activeList", "")

    def activate(self, name):
        return self.callNR("activate", name)

    def deactivate(self, name):
        return self.callNR("deactivate", name)

    def getDescription(self, name):
        return self.callStr("getDescription", name)

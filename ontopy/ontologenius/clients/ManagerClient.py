import rospy

from .ClientBase import ClientBase

class ManagerClient(ClientBase):

    def __init__(self):
        ClientBase.__init__(self, "manage")

    def list(self):
        return self.call("list", "")

    def add(self, name):
        return self.callNR("add", name)

    def copy(self, dest_name, src_name):
        return self.callNR("copy", dest_name + "=" + src_name)

    def delete(self, name):
        return self.callStr("delete", name)

    def getDifference(self, onto1, onto2, concept):
        return self.callNR("difference", onto1 + "|" + onto2 + "|" + concept)

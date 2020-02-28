import rospy

from .clients import *
from .OntologyManipulator import OntologyManipulator

class OntologiesManipulator(ManagerClient):
    def __init__(self):
        self.manipulators = {}

    def waitInit(self, timeout = -1):
        rospy.wait_for_service("ontologenius/manage", timeout)

    def add(self, name):
        if name in self.manipulators:
            return True
        else:
            if ManagerClient.add(name) == False:
                return False
            else:
                self.manipulators[name] = OntologyManipulator(name)
                return True

    def copy(self, dest_name, src_name):
        if dest_name in self.manipulators:
            return True
        else:
            if ManagerClient.copy(dest_name, src_name) == False:
                return False
            else:
                self.manipulators[dest_name] = OntologyManipulator(dest_name)
                return True

    def delete(self, name):
        if name not in self.manipulators:
            return True
        else:
            if ManagerClient.delete(name) == False:
                return False
            else:
                del self.manipulators[name]
                return True

    def setVerbose(self, verose):
        ClientBase.setVerbose(verbose)

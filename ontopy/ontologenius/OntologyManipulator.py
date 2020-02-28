import rospy

from .clients import *

class OntologyManipulator:
    def __init__(self, name):
        self._name = name
        self.individuals = IndividualClient(name)
        self.objectProperties = ObjectPropertyClient(name)
        self.dataProperties = DataPropertyClient(name)
        self.classes = ClassClient(name)
        self.actions = ActionClient(name)
        self.reasoners = ReasonerClient(name)

        service_name = "ontologenius/reasoner"
        if name != '':
            service_name+= "/" + name
        rospy.wait_for_service(service_name)

    def nb(self):
        return self.actions.nb()

    def resetNb(self):
        self.actions.resetNb()

    def close(self):
        return self.actions.close()

    def setVerbose(self, verbose):
        ClientBase.setVerbose(verbose)

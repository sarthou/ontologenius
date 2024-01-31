from .clients import *
from .clientsIndex import *
from .OntologyManipulator import OntologyManipulator
from .OntologyManipulatorIndex import OntologyManipulatorIndex

class OntologiesManipulator(ManagerClient, object):
    """The OntologiesManipulator class allows to create and delete ontologies instances dynamically.
       Its usage is strongly recommended for multi-ontology usage and ensures good thread management.
       This class allows you to set up the theory of mind quickly in a software application.
    """
    def __init__(self):
        """Constructs a manipulator for several instances of ontologies."""
        super(OntologiesManipulator, self).__init__()
        self.manipulators = {}
        self.manipulators_index = {}

    def waitInit(self, timeout = -1):
        """Wait for ontologenius services to be advertised and available for. Blocks until it is.
           timeout(int) is the amount of time to wait for before timing out.
           If timeout is -1 (default), waits until the node is shutdown.
        """
        super(OntologiesManipulator, self).waitInit(timeout)

    def add(self, name):
        """Creates a new instance of OntologyManipulator and OntologyManipulatorIndex identified by the name name(str).
           Returns False if the creation fails. Returns True even if the instance already exists.
        """
        if name in self.manipulators:
            return True
        else:
            if super(OntologiesManipulator, self).add(name) == False:
                return False
            else:
                self.manipulators[name] = OntologyManipulator(name)
                self.manipulators_index[name] = OntologyManipulatorIndex(name)
                return True

    def copy(self, dest_name, src_name):
        """Creates a new instance of OntologyManipulator and OntologyManipulatorIndex identified by the name dest_name(str) that manipulates a copy of 
           the ontology handled by the ontologyManipulator src_name(str).
           Returns False if the copy fails. Returns True even if the instance already exists.
        """
        if dest_name in self.manipulators:
            return True
        else:
            if super(OntologiesManipulator, self).copy(dest_name, src_name) == False:
                return False
            else:
                self.manipulators[dest_name] = OntologyManipulator(dest_name)
                self.manipulators_index[dest_name] = OntologyManipulatorIndex(dest_name)
                return True

    def delete(self, name):
        """Deletes the instance of OntologyManipulator and OntologyManipulatorIndex identified by the name name(str).
           Returns False deletion fails. Returns True even if the instance does not exist.
        """ 
        if name not in self.manipulators:
            return True
        else:
            if super(OntologiesManipulator, self).delete(name) == False:
                return False
            else:
                del self.manipulators[name]
                del self.manipulators_index[name]
                return True

    def get(self, name):
        """Returns an OntologyManipulator object instance named name(str).
           Returns None if no OntologyManipulator instance is named name.
        """
        if name not in self.manipulators:
            return None
        else:
            return self.manipulators[name]

    def getIndex(self, name):
        """Returns an OntologyManipulatorIndex object instance named name(str).
           Returns None if no OntologyManipulatorIndex instance is named name.
        """
        if name not in self.manipulators_index:
            return None
        else:
            return self.manipulators_index[name]

    def setVerbose(self, verbose):
        """If verbose is set to True, the clients will post messages about the failure to call the services and about their restoration."""
        ClientBase.setVerbose(verbose)
        ClientBaseIndex.setVerbose(verbose)

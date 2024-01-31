from .clients import *
from .clientsIndex import *
from .FeederPublisher import FeederPublisher
from .ConversionClient import ConversionClient

class OntologyManipulatorIndex:
    """The OntologyManipulatorIndex class is just an object to access all API ROS abstraction classes so that you can query and manage ontologenius."""

    def __init__(self, name = ''):
        """Constructs an ontology manipulator with.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str). For classic use, do not specify the ontology name name.
        """
        self._name = name
        self.individuals = IndividualIndexClient(name)
        self.objectProperties = ObjectPropertyIndexClient(name)
        self.dataProperties = DataPropertyIndexClient(name)
        self.classes = ClassIndexClient(name)
        self.actions = ActionClient(name)
        self.reasoners = ReasonerClient(name)
        self.feeder = FeederPublisher(name)
        self.sparql = SparqlIndexClient(name)
        self.conversion = ConversionClient(name)

        self.conversion._client.wait()

    def nb(self):
        """Gives the total number (int) of service calls from all ROS clients instances since the last reset."""
        return self.actions.nb() + self.individuals.nb()

    def resetNb(self):
        """Reset the call counter for all instances of ROS clients."""
        self.actions.resetNb()
        self.individuals.resetNb()

    def close(self):
        """Same as the ActionClient closing function. Link all the concepts loaded from files and the Internet. Before closing an ontology, exploration requests are not allowed.
           Returns False if the service call fails.
        """
        return self.actions.close()

    def setVerbose(self, verbose):
        """If verbose is set to True, the clients will post messages about the failure to call the services and about their restoration."""
        ClientBase.setVerbose(verbose)
        ClientBaseIndex.setVerbose(verbose)

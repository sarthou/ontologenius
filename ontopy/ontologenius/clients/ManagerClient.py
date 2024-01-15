from .ClientBase import ClientBase

class ManagerClient(ClientBase):
    """The ManagerClient class provides an abstraction ontologenius manage ROS services.
        The manager client is used to manage ontology instances in multi-ontology usage.
        This makes it possible to start new ontologies or to stop old ones, but also to study
        the differences of knowledge between several instances. With the manager client,
        it is easy to develop software using theory of mind.
        This class is based on ClientBase and so ensure a persistent connection with
        the service based on. The persistent connection ensures a minimal response time.
        A reconnection logic is implemented in the event that the persistent connection fails. 
    """
    def __init__(self):
        """Constructs a manager client.
           Can only be used in a multi-ontology mode.
        """
        ClientBase.__init__(self, "manage")

    def waitInit(self, timeout = -1):
        """Wait for ontologenius services to be advertised and available for. Blocks until it is.
           timeout(int) is the amount of time to wait for before timing out.
           If timeout is -1 (default), waits until the node is shutdown.
        """
        self._client.wait(timeout)

    def list(self):
        """Returns the name of the instantiated ontologies (str[])."""
        return self.call("list", "")

    def add(self, name):
        """Create an ontology instance named name(str).s
           Returns False if the service call fails.
        """
        return self.callNR("add", name)

    def copy(self, dest_name, src_name):
        """Create a copy of the ontology instance named src_name(str) with the name dest_name(str).
           Returns False if the service call fails or if the copy fails.
        """
        return self.callBool("copy", dest_name + "=" + src_name)

    def delete(self, name):
        """Delete the instance of the ontology named name(str)
           Returns False if the service call fails.
        """
        return self.callNR("delete", name)

    def getDifference(self, onto1, onto2, concept):
        """Returns the difference of knowledge between onto_1(str) and onto_2(str) regarding the concept (class or individual) concept(str).
           The elements of the returned vector are formated as : [+]concept_from|property|concept_on OR [-]concept_from|property|concept_on.
           An element is positive if it is present in onto_1 and not in onto_2 and negative in reverse.
           The difference in inheritance knowledge between concepts is returned with the property isA.
        """
        return self.call("difference", onto1 + "|" + onto2 + "|" + concept)

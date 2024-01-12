from .ClientBase import ClientBase

class ReasonerClient(ClientBase):
    """The ReasonerClient class provides an abstraction ontologenius reasoner ROS services.
       The reasoner client is used to manage reasoner plugins for an ontology instance.
       This makes it possible to activate or deactivate plugins.
       This class is based on ClientBase and so ensure a persistent connection with the service based on.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs a manager client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str). For classic use, name should be defined as "".
        """
        if name == '':
            ClientBase.__init__(self, "reasoner")
        else:
            ClientBase.__init__(self, "reasoner/" + name)

    def list(self):
        """Returns the name of the plugins available (str[])."""
        return self.call("list", "")

    def activeList(self):
        """Returns the name of the activated plugins (str[])."""
        return self.call("activeList", "")

    def activate(self, name):
        """Activate the plugin named name(str).
           Returns False if the service call fails.
        """
        return self.callNR("activate", name)

    def deactivate(self, name):
        """Deactivate the plugin named name(str).
           Returns False if the service call fails.
        """
        return self.callNR("deactivate", name)

    def getDescription(self, name):
        """Returns the description (str) of the plugin named name."""
        return self.callStr("getDescription", name)

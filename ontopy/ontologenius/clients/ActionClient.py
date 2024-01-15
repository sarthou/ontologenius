from .ClientBase import ClientBase

class ActionClient(ClientBase):
    """The ActionClient class provides an abstraction ontologenius action ROS services.
       This class is based on ClientBase and so ensure a persistent connection with the service based on.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs an action client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name(str) should be defined as ''.
        """
        if name == '':
            ClientBase.__init__(self, "actions")
        else:
            ClientBase.__init__(self, "actions/" + name)

    def close(self):
        """Link all the concepts loaded from files and the Internet.
           Before closing an ontology, exploration requests are not allowed.
           Returns false ontology closure fails or if the service call fails.
        """
        if self.callNR("close", "") == False:
            return False
        else:
            return self.error_code == 0

    def save(self, path):
        """Saves the current ontology in the absolute path(str) path.
           The path(str) parameter must be of the form: my/path/to/ontology.owl
           Returns False if the service call fails.
        """
        return self.callNR("save", path)

    def export(self, path):
        """Exports the current modification tree in the absolute path(str) path.
           The path(str) parameter must be of the form: my/path/to/file.xml
           This function has no effect on non copied ontologies.
           Returns False if the service call fails.
        """
        return self.callNR("export", path)

    def setLang(self, lang):
        """Sets the language of work lang(str).
           Returns False if the service call fails.
        """
        return self.callNR("setLang", lang)

    def getLang(self):
        """Return the working language (str)."""
        return self.callStr("getLang", "")

    def add(self, uri):
        """Load an ontology file (.owl) stored at uri(str) from the internet.
           The Close function should be called after all the desired files have been loaded.
           Returns False if the service call fails.
        """
        return self.callNR("add", uri)

    def fadd(self, file):
        """Load an ontology file (.owl) stored at file(str) from your local computer.
           The Close function should be called after all the desired files have been loaded.
           Returns False if the service call fails.
        """
        return self.callNR("fadd", file)

    def reset(self):
        """Unload all the knowledge previously loaded or learned and reload the default files.
           Returns False if the service call fails.
        """
        return self.callNR("reset", "")

    def clear(self):
        """Unload all the knowledge previously loaded or learned.
           Returns False if the service call fails.
        """
        return self.callNR("clear", "")

from .ClientBase import ClientBase

class OntologyClient(ClientBase):
    """The OntologyClient class provides an abstraction common to all ontologenius exploration ROS services.
       The OntologyClient implements the functions common to every ontologenius exploration.
       This class is based on ClientBase and so ensure a persistent connection with the service based on.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """

    def __init__(self, name):
        """Constructs an ontology client linked to the service ontologenius/name(str)."""
        ClientBase.__init__(self, name)

    def getUp(self, name, depth = -1, selector = ''):
        """Gives all concepts below the one given in the parameter: name(str).
           The optional depth(int) parameter can be set to limit tree propagation to a specific value.
           The default value -1 represents no propagation limitation.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) concept.
           The default value '' represents no restriction on the result.
        """
        param = name
        if selector != '':
            param += " -s " + selector
        if depth >= 0:
            param += " -d " + str(depth)
        return self.call("getUp", param)

    def isA(self, name, base_class):
        """Return true if the concept name(str) is or inherits of the concept base_class(str).
           This function corresponds to checking if class_base is part of the result of the function getUp applies to the concept name.
        """
        res = self.getUp(name, selector = base_class)
        if len(res) == 0:
            return False
        else:
            return True

    def getName(self, name, take_id = True):
        """Gives one of the label (str) of the concept name(str) that is not muted.
           The default take_id(bool) argument can be set to False if you do not want to
           consider the concept identifier as a possible default name.
           The result of this function depends on the setting of the working language.
        """
        param = name
        if take_id == False:
            param += " -i false"
        return self.callStr("getName", param)

    def getNames(self, name, take_id = True):
        """Gives all the labels (str[]) of the concept name(str) excepted the muted ones.
           The default take_id(bool) argument can be set to False if you do not want to
           consider the concept identifier as a possible default name.
           The result of this function depends on the setting of the working language.
        """
        param = name
        if take_id == False:
            param += " -i false"
        return self.call("getNames", param)

    def getEveryNames(self, name, take_id = True):
        """Gives all the labels (str[]) of the concept name(str) even the muted ones.
           The default take_id(bool) argument can be set to False if you do not want to
           consider the concept identifier as a possible default name.
           The result of this function depends on the setting of the working language.
        """
        param = name
        if take_id == False:
            param += " -i false"
        return self.call("getEveryNames", param)

    def find(self, name, take_id = True, selector = ''):
        """Gives all the concepts (str[]) having for label name(str).
           The default take_id(bool) argument can be set to False if you do not want
           to consider the concept identifier as a possible default name.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) concept.
           The default value '' represents no restriction on the result.
           The result of this function depends on the setting of the working language.
        """
        param = name
        if take_id == False:
            param += " -i false"
        if selector != '':
            param += " -s " + selector
        return self.call("find", param)

    def findSub(self, name, take_id = True, selector = ''):
        """Gives all the concepts (str[]) having for label a subset of name(str).
           The default take_id(bool) argument can be set to false if you do not want to
           consider the concept identifier as a possible default name.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) concept.
           The default value '' represents no restriction on the result.
           The result of this function depends on the setting of the working language.
        """
        param = name
        if take_id == False:
            param += " -i false"
        if selector != '':
            param += " -s " + selector
        return self.call("findSub", param)

    def findRegex(self, name, take_id = True, selector = ''):
        """Give all concepts (str[]) with a label matching the regular expression regex(str).
           The default take_id(bool) argument can be set to false if you do not want to consider
           the concept identifier as a possible default name.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) concept.
           The default value '' represents no restriction on the result.
           The result of this function depends on the setting of the working language.
        """
        param = name
        if take_id == False:
            param += " -i false"
        if selector != '':
            param += " -s " + selector
        return self.call("findRegex", param)

    def findFuzzy(self, name, threshold = 0.5, take_id = True, selector = ''):
        """Give all the names of concepts (str[]) with the lowest
           edit distance with parameter name(str).
           The default take_id(bool) argument can be set to false if you do not want to
           consider the concept identifier as a possible default name.
           The optional selector(str) parameter can be set to only get results inheriting from the selector(str) concept.
           The default value '' represents no restriction on the result.
           The result of this function depends on the setting of the working language and
           does not correspond to the concept identifiers but to other labels known by ontologenius.
           The minimum editing distance can be set with the threshold(double) parameter.
           This value corresponds to the number of changes to be made to pass from one
           word to another divided by the length of the comparison word.
        """
        param = name + ' -t ' + str(threshold)
        if take_id == False:
            param += " -i false"
        if selector != '':
            param += " -s " + selector
        return self.call("findFuzzy", param)

    def exist(self, name):
        """Returns True if the concept name(str) exists in the subpart of the ontology 
           managed by the client (i.e. class, individuals, object properties, data properties).
        """
        return self.callStr("exist", name) != ''

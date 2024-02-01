from ..compat.ros import Ontoros, OntoService
import os

from ontologenius.srv import OntologeniusIndexService
if os.environ["ROS_VERSION"] == "1":
    from ontologenius.srv import OntologeniusIndexServiceRequest
else:
    from ontologenius.srv._ontologenius_index_service import OntologeniusIndexService_Request as OntologeniusIndexServiceRequest

class ClientBaseIndex:
    """The ClientBaseIndex class provides an abstraction for any ROS services.
       This class ensures a persistent connection with the service based on indexes.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """
    _verbose = False
    _cpt = 0

    def __init__(self, name):
        """Constructs a ROS client linked to the service name(str)."""
        self._name = name
        self._client = Ontoros.createService('ontologenius/' + self._name, OntologeniusIndexService)
        self.error_code = 0

    def nb(self):
        """Gives the total number (int) of service calls from all ClientBaseIndex instances since the last reset."""
        return ClientBaseIndex._cpt

    def resetNb(self):
        """Reset Call Counter for all instances of ClientBaseIndex."""
        ClientBaseIndex._cpt = 0

    def setVerbose(self, verbose):
        """If verbose(bool) is set to True, the clients will post messages about
           the failure to call the services and about their restoration.
        """
        ClientBaseIndex._verbose = verbose

    def call(self, action, param):
        """Call the service set up in the constructor of ClientBaseIndex with the request
           defined with action(str) and param(str) and returns all the results (str[]).
           If the service call fails, the function returns None
        """
        ClientBaseIndex._cpt += 1
        request = OntologeniusIndexServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBaseIndex._verbose)
        if(response is None):
            self.error_code = -1
            return None
        else:
            self.error_code = response.code
            return response.string_values

    def callIndexes(self, action, param):
        """Call the service set up in the constructor of ClientBaseIndex with the request
           defined with action(str) and param(str) and returns all the results (integer[]).
           If the service call fails, the function returns None
        """
        ClientBaseIndex._cpt += 1
        request = OntologeniusIndexServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBaseIndex._verbose)
        if(response is None):
            self.error_code = -1
            return None
        else:
            self.error_code = response.code
            return response.index_values

    def callStr(self, action, param):
        """Call the service set up in the constructor of ClientBaseIndex with the request
           defined with action(str) and param(str) and returns all the first result (str).
           If the service call fails, the function returns None
        """
        ClientBaseIndex._cpt += 1
        request = OntologeniusIndexServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBaseIndex._verbose)
        if(response is None):
            self.error_code = -1
            return None
        else:
            self.error_code = response.code
            if len(response.string_values) > 0:
                return response.string_values[0]
            else:
                return ''

    def callIndex(self, action, param):
        """Call the service set up in the constructor of ClientBaseIndex with the request
           defined with action(str) and param(str) and returns all the first result (integer).
           If the service call fails, the function returns None
        """
        ClientBaseIndex._cpt += 1
        request = OntologeniusIndexServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBaseIndex._verbose)
        if(response is None):
            self.error_code = -1
            return None
        else:
            self.error_code = response.code
            if len(response.index_values) > 0:
                return response.index_values[0]
            else:
                return 0

    def callNR(self, action, param):
        """Call the service set up in the constructor of ClientBaseIndex with the
           request defined with action(str) and param(str).
           If the service call fails, the function returns False
        """
        ClientBaseIndex._cpt += 1
        request = OntologeniusIndexServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBaseIndex._verbose)
        if(response is None):
            self.error_code = -1
            return False
        else:
            self.error_code = response.code
            return True

    def callBool(self, action, param):
        """Call the service set up in the constructor of ClientBaseIndex with the
           request defined with action(str) and param(str).
           Returns False if the service call fails or the result code of the
           service is different from SUCCESS.
        """
        ClientBaseIndex._cpt += 1
        request = OntologeniusIndexServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBaseIndex._verbose)
        if(response is None):
            self.error_code = -1
            return False
        else:
            self.error_code = response.code
            return response.code == 0

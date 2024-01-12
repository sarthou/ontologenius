from ..compat.ros import Ontoros
import os

from ontologenius.srv import OntologeniusService
if os.environ["ROS_VERSION"] == "1":
    from ontologenius.srv import OntologeniusServiceRequest
else:
    from ontologenius.srv._ontologenius_service import OntologeniusService_Request as OntologeniusServiceRequest


class ClientBase:
    """The ClientBase class provides an abstraction for any ROS services.
       This class ensures a persistent connection with the service based on.
       The persistent connection ensures a minimal response time.
       A reconnection logic is implemented in the event that the persistent connection fails. 
    """
    _verbose = False
    _cpt = 0

    def __init__(self, name):
        """Constructs a ROS client linked to the service name(str)."""
        self._name = name
        self.error_code = 0
        self._client = Ontoros.createService('ontologenius/' + self._name, OntologeniusService)

    def nb(self):
        """Gives the total number (int) of service calls from all ClientBase instances since the last reset."""
        return ClientBase._cpt

    def resetNb(self):
        """Reset Call Counter for all instances of ClientBase."""
        ClientBase._cpt = 0

    def setVerbose(self, verbose):
        """If verbose(bool) is set to True, the clients will post messages about
           the failure to call the services and about their restoration.
        """
        ClientBase._verbose = verbose

    def call(self, action, param):
        """Call the service set up in the constructor of ClientBase with the request
           defined with action(str) and param(str) and returns all the results (str[]).
           If the service call fails, the function returns None
        """
        ClientBase._cpt += 1
        request = OntologeniusServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBase._verbose)
        if(response is None):
            self.error_code = -1
            return None
        else:
            self.error_code = response.code
            return response.values

    def callStr(self, action, param):
        """Call the service set up in the constructor of ClientBase with the request
           defined with action(str) and param(str) and returns all the first result (str).
           If the service call fails, the function returns None
        """
        ClientBase._cpt += 1
        request = OntologeniusServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBase._verbose)
        if(response is None):
            self.error_code = -1
            return None
        else:
            self.error_code = response.code
            if len(response.values) > 0:
                return response.values[0]
            else:
                return ''

    def callNR(self, action, param):
        """Call the service set up in the constructor of ClientBase with the
           request defined with action(str) and param(str).
           If the service call fails, the function returns False
        """
        ClientBase._cpt += 1
        request = OntologeniusServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBase._verbose)
        if(response is None):
            self.error_code = -1
            return False
        else:
            self.error_code = response.code
            return True

    def callBool(self, action, param):
        """Call the service set up in the constructor of ClientBase with the
           request defined with action(str) and param(str).
           Returns False if the service call fails or the result code of the
           service is different from SUCCESS.
        """
        ClientBase._cpt += 1
        request = OntologeniusServiceRequest(action = action, param = param)
        response = self._client.call(request, ClientBase._verbose)
        if(response is None):
            self.error_code = -1
            return False
        else:
            self.error_code = response.code
            return response.code == 0

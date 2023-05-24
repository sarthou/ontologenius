import rospy

from ontologenius.srv import OntologeniusService

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
        self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)

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
        try:
            response = self._client(action, param)
            return response.values
        except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
            if ClientBase._verbose == True:
                print("Failure to call ontologenius/" + self._name)
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print("Restored ontologenius/" + self._name)
                return response.values
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                if ClientBase._verbose == True:
                    print("Failure of service restoration")
                return None

    def callStr(self, action, param):
        """Call the service set up in the constructor of ClientBase with the request
           defined with action(str) and param(str) and returns all the first result (str).
           If the service call fails, the function returns None
        """
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            if len(response.values) > 0:
                return response.values[0]
            else:
                return ''
        except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
            if ClientBase._verbose == True:
                print("Failure to call ontologenius/" + self._name)
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print("Restored ontologenius/" + self._name)
                if len(response.values) > 0:
                    return response.values[0]
                else:
                    return ''
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                if ClientBase._verbose == True:
                    print("Failure of service restoration")
                return None

    def callNR(self, action, param):
        """Call the service set up in the constructor of ClientBase with the
           request defined with action(str) and param(str).
           If the service call fails, the function returns False
        """
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            return True
        except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
            if ClientBase._verbose == True:
                print("Failure to call ontologenius/" + self._name)
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print("Restored ontologenius/" + self._name)
                return True
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                if ClientBase._verbose == True:
                    print("Failure of service restoration")
                return False

    def callBool(self, action, param):
        """Call the service set up in the constructor of ClientBase with the
           request defined with action(str) and param(str).
           Returns False if the service call fails or the result code of the
           service is different from SUCCESS.
        """
        ClientBase._cpt += 1
        try:
            response = self._client(action, param)
            return response.code == 0
        except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
            if ClientBase._verbose == True:
                print("Failure to call ontologenius/" + self._name)
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusService, True)
            try:
                response = self._client(action, param)
                if ClientBase._verbose == True:
                    print("Restored ontologenius/" + self._name)
                return response.code == 0
            except (rospy.ServiceException, rospy.exceptions.TransportTerminated) as e:
                if ClientBase._verbose == True:
                    print("Failure of service restoration")
                return False

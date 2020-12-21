import rospy

from ontologenius.srv import OntologeniusSparqlService

class SparqlClient:

    def __init__(self, name):
        self._name = 'sparql'
        if name != '':
            self._name = self._name + '/' + name
        self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusSparqlService, True)


    def call(self, query):
        try:
            response = self._client(query)
            return response.results
        except rospy.ServiceException as e:
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusSparqlService, True)
            try:
                response = self._client(query)
                return response.results
            except rospy.ServiceException as e:
                return None

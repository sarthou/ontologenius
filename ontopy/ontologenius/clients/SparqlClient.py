import rospy

from ontologenius.srv import OntologeniusSparqlService

class SparqlClient:
    """The SparqlClient class provides a ROS service to explore ontologenius with SPARQL-like queries.
       The variables start with the symbol ? (e.g. ?my_var) and each triplet is separated by a comma.
    """

    def __init__(self, name):
        """Constructs a sparql client.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str). For classic use, name should be defined as ''.
        """
        self._name = 'sparql'
        if name != '':
            self._name = self._name + '/' + name
        self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusSparqlService, True)


    def call(self, query):
        try:
            response = self._client(query)
            return (response.names, response.results)
        except rospy.ServiceException as e:
            self._client = rospy.ServiceProxy('ontologenius/' + self._name, OntologeniusSparqlService, True)
            try:
                response = self._client(query)
                return (response.names, response.results)
            except rospy.ServiceException as e:
                return None

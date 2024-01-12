from ..compat.ros import Ontoros, OntoService
import os

from ontologenius.srv import OntologeniusSparqlService
if os.environ["ROS_VERSION"] == "1":
    from ontologenius.srv import OntologeniusSparqlServiceRequest
else:
    from ontologenius.srv._ontologenius_sparql_service import OntologeniusSparqlService_Request as OntologeniusSparqlServiceRequest

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
        self._client = Ontoros.createService('ontologenius/' + self._name, OntologeniusSparqlService)

    def call(self, query):
        request = OntologeniusSparqlServiceRequest(query = query)
        response = self._client.call(request)
        if(response is None):
            return None
        else:
            return (response.names, response.results)

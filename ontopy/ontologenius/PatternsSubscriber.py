from .compat.ros import Ontoros
import os

from ontologenius.msg import OntologeniusSubscriptionAnswer
from ontologenius.srv import OntologeniusSubscription, OntologeniusUnsubscription

if os.environ["ROS_VERSION"] == "1":
    from ontologenius.srv import OntologeniusSubscriptionRequest
    from ontologenius.srv import OntologeniusUnsubscriptionRequest
else:
    from ontologenius.srv._ontologenius_subscription import OntologeniusSubscription_Request as OntologeniusSubscriptionRequest
    from ontologenius.srv._ontologenius_unsubscription import OntologeniusUnsubscription_Request as OntologeniusUnsubscriptionRequest

class PatternsSubscriber:
    """The PatternsSubscriber class provides an abstraction ontologenius subscription mechanism.
       Querying a knowledge base is interesting but when one is waiting a given fact, pulling 
       the knowledge base can be an issue. To solve that, Ontologenius provide a subscription
       mechanism allowing to subscribe not one only to given facts but also to patterns.
    """

    def __init__(self, name):
        """Constructs a PatternsSubscriber.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name(str) should be defined as ''.
        """
        self._name = name

        self._ids = {}

        sub_topic_name = 'ontologenius/subscription_answer'
        if self._name != '':
            sub_topic_name += '/' + self._name
        self._answer_sub = Ontoros.createSubscriber(sub_topic_name, OntologeniusSubscriptionAnswer, self.patternCallback)

        client_sub_name = 'ontologenius/subscribe'
        if self._name != '':
            client_sub_name += '/' + self._name
        self._sub_client = Ontoros.createService(client_sub_name, OntologeniusSubscription)

        client_unsub_name = 'ontologenius/unsubscribe'
        if self._name != '':
            client_unsub_name += '/' + self._name
        self._unsub_client = Ontoros.createService(client_unsub_name, OntologeniusUnsubscription)

    def __del__(self):
        ids = self._ids.keys()
        for id in ids:
            self.cancel(id)

        self._answer_sub.unregister()

    def subscribe(self, pattern, callback, count = -1):
        """Subscribes to a given pattern linked to a callback.
           The parameter count can be set to limit the subscription. 
           Default parameter -1 corresponds to an unlimited subscription.
           This function returns the subscription id. This later is only used to manually unsubscribe.
        """
        request = OntologeniusSubscriptionRequest(data = pattern, count = count)
        response = self._sub_client.call(request, False)
        if(response):
            self._ids[response.id] = callback
            return response.id
        else:
            return None
        
    def cancel(self, id):
        """Manualy unsubscribe from a pattern using the corresponding subscription id.
           Return True if the unsubscription suceeded.
        """
        request = OntologeniusUnsubscriptionRequest(id = id)
        response = self._unsub_client.call(request, False)
        if(response):
            if(response.id == id):
                self._ids.pop(id)
                return True
            else:
                return False
        else:
            return False

    def patternCallback(self, msg):
        if msg.id in self._ids.keys():
            self._ids[msg.id](msg.data)
            if(msg.last):
                self._ids.pop(msg.id)
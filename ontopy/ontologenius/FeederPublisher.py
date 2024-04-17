from .compat.ros import Ontoros

import os

from std_msgs.msg import String
from ontologenius.msg import OntologeniusStampedString as StampedString
from ontologenius.msg import OntologeniusTimestamp as Timestamp

import time
import random
from datetime import datetime

class FeederPublisher:
    """The FeederPublisher class provides an abstraction ontologenius feeder(insert) ROS topic.
       Working in a closed world can be interesting, but with ontologenius, you can also choose to 
       work in an open world by adding and modifying the agent's knowledge base during its operation.
       The feeder publisher is used to insert and delete knowledge dynamically.
       The feeding process is asynchronous and therefore does not guarantee any response time.
       It still provides functions to synchronize if you have applications where you have to query the ontology right after having modified it.
       All modifications can be time-stamped for advanced uses using the republication mechanism.
    """

    def __init__(self, name):
        """Constructs a FeederPublisher.
           Can be used in a multi-ontology mode by specifying the name of the ontology name(str).
           For classic use, name(str) should be defined as ''.
        """
        self._name = name
        pub_topic_name = 'ontologenius/insert'
        if self._name != '':
            pub_topic_name += '/' + self._name
        self._pub = Ontoros.createPublisher(pub_topic_name, String, queue_size=1000)
        pub_topic_name = 'ontologenius/insert_stamped'
        if self._name != '':
            pub_topic_name += '/' + self._name
        self._stamped_pub = Ontoros.createPublisher(pub_topic_name, StampedString, queue_size=1000)
        sub_topic_name = 'ontologenius/end'
        if self._name != '':
            sub_topic_name += '/' + self._name
        self._commit_sub = Ontoros.createSubscriber(sub_topic_name, String, self.commitCallback)
        notif_topic_name = 'ontologenius/reasoner_notifications'
        if self._name != '':
            notif_topic_name += '/' + self._name
        self._notif_sub = Ontoros.createSubscriber(notif_topic_name, String, self._notifCallback)
        self.user_notif_callback = None
        self._updated = False
        random.seed()
        self._commit_nb = random.randint(1, 100000)

    def __del__(self):
        self._commit_sub.unregister()

    def addObjectProperty(self, concept_from, property, concept_on, stamp = None):
        """Adds the fact that concept_from(str) is linked with concept_on(str) by the property property(str).
           At least concept_from or concept_on must be already known to the system.
           If one of them is unknown, it will be automatically created. The property can be unknown before calling this function.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[add]' + concept_from + '|' + property + '|' + concept_on
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def addDataProperty(self, concept_from, property, type, data, stamp = None):
        """Adds the fact that concept_from(str) is linked to the data data(str) of type type(str) by the property property(str).
           concept_from must be already known to the system. The property can be unknown before calling this function.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[add]' + concept_from + '|' + property + '|' + type + '#' + data
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def addInheritage(self, child, mother, stamp = None):
        """Adds the inheratage : child(str) is a mother(str). child and mother could by a class, an individual or a property.
           At least child or mother must be already known to the system. If one of them is unknown, it will be automatically created.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[add]' + child + '|+|' + mother
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def addLanguage(self, concept_from, lang, name, stamp = None):
        """Adds the label name(str) in the language lang(str) the class, individual, or property concept_from(str).
           concept_from must be already known to the system.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[add]' + concept_from + '|@' + lang + '|' + name
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def addConcept(self, concept_from, stamp = None):
        """Adds the class or individual concept_from(str).
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[add]' + concept_from + '|'
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def removeProperty(self, concept_from, property, stamp = None):
        """Removes the fact that concept_from(str) is linked to any object by the property property(str).
           After this action, knowledge of the property is not removed.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[del]' + concept_from + '|' + property + '|_'
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def removeObjectProperty(self, concept_from, property, concept_on, stamp = None):
        """Removes the fact that concept_from(str) is linked with concept_on(str) by the property property(str).
           After this action, knowledge of the property is not removed.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[del]' + concept_from + '|' + property + '|' + concept_on
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def removeDataProperty(self, concept_from, property, type, data, stamp = None):
        """Removes the fact that concept_from(str) is linked to the data data(str) of type type(str) by the property property(str).
           After this action, knowledge of the property is not removed.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[del]' + concept_from + '|' + property + '|' + type + '#' + data
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def removeInheritage(self, concept_from, concept_on, stamp = None):
        """Removes the inheratage : concept_from(str) is a concept_on(str).
        concept_from and concept_on could by a class, an individual or a property.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[del]' + concept_from + '|+|' + concept_on
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def removeLanguage(self, concept_from, lang, name, stamp = None):
        """Removes the label name(str) in the language lang(str) the class, individual, or property concept_from(str).
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[del]' + concept_from + '|@' + lang + '|' + name
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def removeConcept(self, concept_from, stamp = None):
        """Removes the class or individual concept_from(str).
           All inheritance, properties, and labels applied to concept_from are also removed.
           If the time stamp stamp (rostime) is not defined, the function takes the current ROS time as the time stamp.
        """
        msg = '[del]' + concept_from + '|'
        if stamp == None:
            stamp = Ontoros.getRosTime()
        self._publish_stamped(msg, stamp)

    def getNumSubscribers(self):
        """Returns the number of subscribers (int) that are currently connected to the internal ROS publisher."""
        return self._pub.getNumSubscribers()

    def getNumPublishers(self):
        return self._commit_sub.getNumPublishers()

    def waitConnected(self):
        """Blocks while no subscribers are currently connected to the internal ROS publisher."""
        while not Ontoros.isShutdown() and self.getNumSubscribers() == 0 and self.getNumPublishers() == 0:
            Ontoros.spin_once()

    def waitUpdate(self, timeout = 100000000):
        """Waits until all changes have been applied.
           The default parameter timeout(int) is the expiration time in milliseconds. The default value is of 100 seconds.
           Returns False if the function returns on a timeout.
        """
        self._updated = False

        start_time = datetime.now()
        self._sendNop()

        while not Ontoros.isShutdown() and not self._updated and (self.millis_interval(start_time, datetime.now()) < timeout) :
            Ontoros.spin_once()

        if self._updated == True:
            return True
        else:
            return False

    def commitAuto(self, timeout = 100000000):
        """Saves all the modifications from the previous commit and waits until all changes have been applied.
           The default parameter timeout(int) is the expiration time in milliseconds. The default value is of 100 seconds.
           Returns the commit id (str) and an empty string if the function returns on a timeout.
           This function can only be used on a copied ontology.
        """
        commit_name = str(self._commit_nb)
        self._commit_nb = self._commit_nb + 1

        if self.commit(commit_name, timeout):
            return commit_name
        else:
            return ''

    def commit(self, commit_name, timeout = 100000000):
        """Saves all the modifications from the previous commit with a specific id commit_name (str) and waits until all changes have been applied.
           The default parameter timeout(int) is the expiration time in milliseconds. The default value is of 100 seconds.
           Returns False if the function returns on a timeout.
           This function can only be used on a copied ontology.
        """
        self._updated = False
        msg = '[commit]' + commit_name + '|'

        start_time = datetime.now()
        self._publish_stamped(msg, Ontoros.getRosTime())

        while (not Ontoros.isShutdown()) and (not self._updated) and ((self.millis_interval(start_time, datetime.now()) < timeout)) :
            Ontoros.spin_once()

        if self._updated == True:
            return True
        else:
            return False

    def checkout(self, commit_name, timeout = 100000000):
        """Apply the necessary changes to return to the specified commit_name (str) and waits until all changes have been applied.
           The default parameter timeout(int) is the expiration time in milliseconds. The default value is of 100 seconds.
           Returns False if the function returns on a timeout.
           This function can only be used on a copied ontology.
        """
        self._updated = False

        start_time = datetime.now()
        self._publish_stamped('[checkout]' + commit_name + '|', Ontoros.getRosTime())

        while not Ontoros.isShutdown() and not self._updated and (self.millis_interval(start_time, datetime.now()) < timeout) :
            Ontoros.spin_once()

        if self._updated == True:
            return True
        else:
            return False
        
    def registerNotificationCallback(self, callback):
        """Register a callback function to get notifications from the reasoners.
           callback is the callback function taking a string.
        """
        self.user_notif_callback = callback

    def _sendNop(self):
        self._publish_stamped('[nop]nop|', Ontoros.getRosTime())

    def _publish(self, data):
        self._pub.publish(data)

    def _publish_stamped(self, data, stamp):
        stamp_onto = stamp
        if not isinstance(stamp_onto,Timestamp) : 
            stamp_onto = Ontoros.getStamp(stamp_onto)
        msg = StampedString(data = data, stamp = stamp_onto)
        self._stamped_pub.publish(msg)

    def commitCallback(self, data):
        if data.data == 'end':
            self._updated = True

    def _notifCallback(self, msg):
        if self.user_notif_callback:
            self.user_notif_callback(msg.data)

    def millis_interval(self, start, end):
        diff = end - start
        millis = diff.days * 24 * 60 * 60 * 1000
        millis += diff.seconds * 1000
        millis += diff.microseconds / 1000
        return millis

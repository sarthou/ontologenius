#!/usr/bin/env python

import rospy
import requests

from ontologenius.srv import REST
import std_msgs.msg
from ontologenius.msg import HTTP_headers


def get_resource(url, headers):
    """
    Gets a resource at the given URL
    :param url: URL of the resource to get
    :return: (string) Body of the HTTP response
    """
    try:
        response = None
        response_code = None
        rospy.loginfo("[REST.get_resource()] GET " + url)

        req = requests.get(url, timeout=20)

        if req.status_code != requests.codes.ok:
            req.raise_for_status()
        if req.status_code == requests.codes.ok:
            response = req.text

        response_code = req.status_code
        return response_code, response

    except requests.exceptions.ConnectionError as err:
        rospy.logwarn("[REST.get_resource()] Connection error (network problem): " + str(err))
    except requests.exceptions.Timeout as err:
        rospy.logwarn("[REST.get_resource()] Timeout error: " + str(err))
    except requests.exceptions.HTTPError as err:
        rospy.logwarn("[REST.get_resource()] HTTP error: " + str(err))
    except requests.exceptions.RequestException as err:
        rospy.logwarn("[REST.get_resource()] Request error: " + str(err))


def delete_resource(url, headers):
    """
    Removes a resource at the given URL
    :param url: URL of the resource to remove
    :return: None
    """
    try:
    response_code = None
        rospy.loginfo("[REST.delete_resource()] DELETE " + url)

        req = requests.delete(url, headers=headers, timeout=8)

        if req.status_code != requests.codes.ok:
            req.raise_for_status()

	response_code = req.status_code
        return response_code, None

    except requests.exceptions.ConnectionError as err:
        rospy.logwarn("[REST.delete_resource()] Connection error (network problem): " + str(err))
    except requests.exceptions.Timeout as err:
        rospy.logwarn("[REST.delete_resource()] Timeout error: " + str(err))
    except requests.exceptions.HTTPError as err:
        rospy.logwarn("[REST.delete_resource()] HTTP error: " + str(err))
    except requests.exceptions.RequestException as err:
        rospy.logwarn("[REST.delete_resource()] Request error: " + str(err))


def post_resource(url, headers, payload=None):
    """
    :param url: The URL where to put the new resource
    :param headers: The header of the POST request
    :param payload: The payload of the request
    :return: (int, string) Response code of the HTTP request, Response body
    """
    response_code = None
    response = None
    try:

        rospy.loginfo("[REST.post_resource()] POST " + url)
        if payload is not None:
            req = requests.post(url, headers=headers, data=payload, timeout=8)
        else:
            req = requests.post(url, headers=headers, timeout=8)
        if not (req.status_code == requests.codes.created or req.status_code == requests.codes.ok):
            req.raise_for_status()
            rospy.logwarn(str(req.status_code))
        else:
            response_code = req.status_code
            response = req.text

    except requests.exceptions.ConnectionError as err:
        rospy.logwarn("[REST.post_resource()] Connection error (network problem): " + str(err))
    except requests.exceptions.Timeout as err:
        rospy.logwarn("[REST.post_resource()] Timeout error: " + str(err))
    except requests.exceptions.HTTPError as err:
        rospy.logwarn("[REST.post_resource()] HTTP error: " + str(err))
    except requests.exceptions.RequestException as err:
        rospy.logwarn("[REST.post_resource()] Request error: " + str(err))

    return response_code, response

def put_in_resource(url, header, payload=None):
    """
    Put a payload in the resource at the given URL
    :param url: URL of the resource to modify
    :param payload: The content we put in resource
    :return: None
    """
    try:
        rospy.loginfo("[REST.put_in_resource()] PUT " + payload + " in " + url)
        req = requests.put(url, data=payload, headers=header, timeout=8)
        if req.status_code != requests.codes.ok:
            req.raise_for_status()
        else:
            response_code = req.status_code

    except requests.exceptions.ConnectionError as err:
        rospy.logwarn("[REST.put_in_resource()] Connection error (network problem): " + str(err))
    except requests.exceptions.Timeout as err:
        rospy.logwarn("[REST.put_in_resource()] Timeout error: " + str(err))
    except requests.exceptions.HTTPError as err:
        rospy.logwarn("[REST.put_in_resource()] HTTP error: " + str(err))
    except requests.exceptions.RequestException as err:
        rospy.logwarn("[REST.put_in_resource()] Request error: " + str(err))

    return response_code, ""

def handle_http(req):
    """
    Generic interface for http request
    :param req:
    :return: (int, string) Response code of the HTTP request, Response body
    """
    response = None
    response_code = None
    URL = req.URL
    names = req.headers.names.split(' ')
    values = req.headers.values.split(' ')
    headers = {}
    i = 0

    while i < len(names):
        headers.update({names[i]: values[i]})
        i = i + 1

    if req.method == "GET":
        response_code, response = get_resource(URL, headers)
    elif req.method == "DELETE":
        response_code, response = delete_resource(URL, headers)
    elif req.method == "POST":
        response_code, response = post_resource(URL, headers, req.body)
    elif req.method == "PUT":
        response_code, response = put_in_resource(URL, headers, req.body)

    if response_code != requests.codes.ok:
        return( None, -1)
    if response_code == requests.codes.ok:
        return(response, 0)

def http_resquest():
    rospy.init_node('ontologenius_rest', anonymous=True)

    rospy.Service('ontologenius/rest', REST, handle_http)
    print "[ INFO] ready to make http request"
    rospy.spin()

if __name__ == "__main__":
    http_resquest()

#! /usr/bin/env python


class ArTagReader(object):
    def __init__(self):
        self.markers = []  # list of markers (update in real time)
        # list of markers saved (update only if update() is called)
        self.saved_markers = []

    def callback(self, msg):
        self.markers = msg.markers

    def update(self):
        self.saved_markers = self.markers

    def get_tag(self, tag):
        """ Returns the marker with id# == tag """
        for marker in self.saved_markers:
            if marker.id == int(tag):
                result = PoseStamped()
                result.pose = marker.pose
                result.header = marker.header
                return result
        return None

    def get_list(self):
        """ Returns the list of saved markers """
        return self.saved_markers

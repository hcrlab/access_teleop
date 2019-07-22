#!/usr/bin/env python

import rospy
import os
import pickle

class Database(object):
    def __init__(self):
        # actions and their offsets
        self._actions = {
          "RLAD": [],  # right leg adduction
          "RLAB": [],  # right leg abduction
          "LLAD": [],  # left leg adduction
          "LLAB": []   # left leg abduction
        }
        # file path of the database
        script_path = os.path.abspath(__file__)
        script_dir = os.path.split(script_path)[0]
        self.db_path = os.path.join(script_dir, "action_db.p")

    def get(self, name):
        if name in self._actions:
            return self._actions[name]
        else:
            return None

    def add(self, name, offset):
        ## if name not in self._actions:
        # always overwrite the previous entry
        self._actions[name] = []
        self._actions[name].append(offset)

    def delete(self, name):
        if name in self._actions:
            del self._actions[name]

    def list(self):
        return self._actions.keys()

    def load(self):
        try:
            with open(self.db_path, 'r') as f:
                self._actions = pickle.load(f)
        except EOFError as eof_e:
            rospy.logwarn('Error when reading the file: {}'.format(eof_e))
        except IOError as io_e:
            rospy.logwarn('No storage information: {}'.format(io_e))

    def save(self):
        try:
            with open(self.db_path, 'w') as f:
                pickle.dump(self._actions, f)
        except IOError as e:
            rospy.logwarn('No storage information: {}'.format(e))
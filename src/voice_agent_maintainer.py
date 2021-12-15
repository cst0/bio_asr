#!/usr/bin/env python3

import rospy
import rospkg
import os
import shutil
import tempfile

from typing import Dict, List

from bio_asr.srv import AddKnownAgent, AddKnownAgentRequest, AddKnownAgentResponse
from bio_asr.srv import GetKnownAgents, GetKnownAgentsRequest, GetKnownAgentsResponse
from bio_asr.srv import SaveKnownAgents, SaveKnownAgentsRequest, SaveKnownAgentsResponse

class VoiceAgentMaintainer:
    def __init__(self, hz=1):
        rospy.Service('add_known_agent',   AddKnownAgent,   self.handle_add_known_agent)
        rospy.Service('get_known_agents',  GetKnownAgents,  self.handle_get_known_agents)
        rospy.Service('save_known_agents', SaveKnownAgents, self.handle_save_known_agents)
        self.known_agent_references = {}

        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        self.unk_count = 0
        self.comparison_dir = os.path.join(path, "data", "known_agents")

        self.populate_known_agent_references()

    def handle_add_known_agent(self, req:AddKnownAgentRequest):
        name = 'unk_' + str(self.unk_count)
        if req.identity_known:
            name = req.agent_name
        else:
            name += str(self.unk_count)
            self.unk_count += 1

        file_path = req.comparison_file_path
        assert type(file_path) is str
        assert len(file_path) != 0

        resp = AddKnownAgentResponse()
        if name in self.known_agent_references.keys():
            if not req.replace_data_if_conflict:
                rospy.logwarn('You requested to add an agent that was already known, and data replacing is off.')
                resp.success = False
                return resp

        if not os.path.exists(file_path):
            rospy.logwarn('Path does not appear to exist')
            resp.success = False
            return resp

        tmp = tempfile.NamedTemporaryFile(suffix='.mp3')
        shutil.copy(file_path, tmp.name)

        self.known_agent_references[name] = tmp.name

    def handle_get_known_agents(self, req:GetKnownAgentsRequest):
        del req
        resp = GetKnownAgentsResponse()
        keys = self.known_agent_references.keys()
        vals = self.known_agent_references.values()

        resp.agent_names = list(keys)
        resp.comparison_file_paths = list(vals)

        return resp

    def handle_save_known_agents(self, req:SaveKnownAgentsRequest):
        del req
        resp = SaveKnownAgentsResponse()
        return resp

    def populate_known_agent_references(self):
        dir_out = os.listdir(self.comparison_dir)
        for out in dir_out:
            print(out)
            if out.endswith('.mp3'):
                assert type(out) is str
                # xyz_1.mp3 -> xyz
                name = ''.join(out.replace('_','').split('.')[:-1])
                self.known_agent_references[name] = os.path.join(self.comparison_dir, out)
        print(self.known_agent_references)

def main():
    rospy.init_node('voice_agent_maintainer')
    vam = VoiceAgentMaintainer()
    rospy.spin()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rospy
import rospkg
import os
import tempfile
from bio_asr.msg import AudioBatch, NotifyFileUsage
from pydub import AudioSegment
from typing import List

class BatchToTempfile:
    def __init__(self, delete_timeout=60):
        self.delete_timeout = 60
        self.sub_audio_batch = rospy.Subscriber(
            "audio/audio_batched", self.handle_audio_batch, queue_size=10
        )

        rp = rospkg.RosPack()
        path = rp.get_path('bio_asr')
        helpers_dir = os.path.join(path, 'helpers')

        self.mp3 = AudioSegment.from_mp3(os.path.join(helpers_dir, 'silent.mp3'))
        #self.wav = AudioSegment.from_mp3(os.path.join(helpers_dir, 'silent.wav'))

        self.files_list:List[str] = []
        self.file_times:List[rospy.Time]

    def handle_audio_batch(self, batch:AudioBatch):
        assert type(self.mp3) is AudioSegment
        assert batch.data is not None

        recording = self.mp3
        for point in batch.data:
            this_mp3 = self.mp3
            this_mp3._data = point
            recording += this_mp3

        tmp = tempfile.NamedTemporaryFile('r+', suffix='.mp3')
        self.files_list.append(tmp.name)
        self.file_times.append(rospy.Time.now())

def main():
    rospy.init_node("audiobatch_to_tempfile", anonymous=False)
    btt = BatchToTempfile()

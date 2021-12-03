#!/usr/bin/env python3

import rospy
import rospkg
import os
import tempfile
from bio_asr.msg import AudioBatch, NotifyFileUsage
from pydub import AudioSegment
from typing import List, Union

class BatchToTempfile:
    def __init__(self, delete_timeout=60):
        self.delete_timeout = 60
        self.sub_audio_batch = rospy.Subscriber(
            "audio/audio_batched", AudioBatch, self.handle_audio_batch, queue_size=10
        )
        self.sub_file_use = rospy.Subscriber(
            "audio/audio_file_locks", NotifyFileUsage, self.handle_file_usage, queue_size=10
        )

        self.pub_file_use = rospy.Publisher("audio/audio_file_updates", NotifyFileUsage, queue_size=1)

        rp = rospkg.RosPack()
        path = rp.get_path('bio_asr')
        helpers_dir = os.path.join(path, 'helpers')

        self.mp3 = AudioSegment.from_mp3(os.path.join(helpers_dir, 'silent.mp3'))
        self.wav = AudioSegment.from_mp3(os.path.join(helpers_dir, 'silent.wav'))

        self.files_list:List[tempfile._TemporaryFileWrapper]
        self.files_strs:List[str]
        self.file_times:List[rospy.Time]
        self.file_user:List[Union[int,None]]

        self.timeout_checker = rospy.Timer(rospy.Duration(1/2), self.handle_delete_timeout)

    def handle_audio_batch(self, batch:AudioBatch):
        assert type(self.mp3) is AudioSegment
        assert batch.data is not None

        recording = self.mp3
        for point in batch.data:
            this_mp3 = self.mp3
            this_mp3._data = point
            recording += this_mp3

        tmp = tempfile.NamedTemporaryFile('r+', suffix='.mp3')
        self.files_list.append(tmp)
        self.files_strs.append(tmp.name)
        self.file_times.append(rospy.Time.now())

        nfu = NotifyFileUsage()
        nfu.path = tmp.name
        nfu.action = nfu.ACTION_AVAILABLE
        self.pub_file_use.publish(nfu)

    def handle_file_usage(self, fileuse:NotifyFileUsage):
        if fileuse.path in self.files_list:
            index = self.files_strs.index(fileuse.path)
            if fileuse.action == fileuse.ACTION_USING:
                self.file_user[index] = fileuse.node_pid
            elif fileuse.action == fileuse.ACTION_RELEASE:
                self.file_user[index] = None

    def handle_delete_timeout(self, event):
        del event
        now = rospy.Time.now()
        for n in range(0, len(self.files_list)):
            # if this time is old enough to delete:
            if (now - self.file_times[n]) > self.delete_timeout:
                # and if nobody is using it:
                if self.file_user[n] is not None:
                    # close the file
                    self.files_list[n].close()
            # if this file isn't old enough to delete, we shouldn't keep
            # checking because the files get added in chronological order
            else:
                return

def main():
    rospy.init_node("audiobatch_to_tempfile", anonymous=False)
    btt = BatchToTempfile()

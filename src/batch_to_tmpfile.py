#!/usr/bin/env python3

import rospy
import rospkg
import os
import tempfile
from bio_asr.msg import AudioBatch, AudioFileNotification
from pydub import AudioSegment


class BatchToTempfile:
    def __init__(self, delete_timeout=60):
        self.delete_timeout = delete_timeout

        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        helpers_dir = os.path.join(path, "helpers")

        self.empty_mp3 = AudioSegment.from_mp3(os.path.join(helpers_dir, "silent.mp3"))
        self.empty_wav = AudioSegment.from_mp3(os.path.join(helpers_dir, "silent.wav"))

        self.files_list = []
        self.files_strs = []
        self.file_times = []
        self.file_user = []

        self.sub_audio_batch = rospy.Subscriber(
            "audio_batched", AudioBatch, self.handle_audio_batch, queue_size=10
        )
        audio_file_use_topic = "audio_file_updates"
        self.pub_file_use = rospy.Publisher(
            audio_file_use_topic, AudioFileNotification, queue_size=1
        )
        self.sub_file_use = rospy.Subscriber(
            audio_file_use_topic,
            AudioFileNotification,
            self.handle_file_usage,
            queue_size=10,
        )

        self.timeout_checker = rospy.Timer(
            rospy.Duration(1 / 2), self.handle_delete_timeout
        )

    def handle_audio_batch(self, batch: AudioBatch):
        assert type(self.empty_mp3) is AudioSegment
        assert batch.data is not None

        recording = self.empty_mp3
        for point in batch.data:
            data = point.data
            this_mp3 = self.empty_mp3
            this_mp3._data = data
            recording += this_mp3

        tmp = tempfile.NamedTemporaryFile("r+", suffix=".mp3")
        recording.export(tmp.name)
        self.files_list.append(tmp)
        self.files_strs.append(tmp.name)
        self.file_times.append(rospy.Time.now())
        self.file_user.append([])

        nfu = AudioFileNotification()
        nfu.path = tmp.name
        nfu.action = nfu.ACTION_AVAILABLE
        self.pub_file_use.publish(nfu)

    def handle_file_usage(self, fileuse: AudioFileNotification):
        if fileuse.path in self.files_list:
            index = self.files_strs.index(fileuse.path)
            if fileuse.action == fileuse.ACTION_USING:
                self.file_user[index].append(fileuse.node_pid)
            elif fileuse.action == fileuse.ACTION_RELEASE:
                if fileuse.node_pid in self.file_user[index]:
                    pid_index = self.file_user[index].index(fileuse.node_pid)
                    del self.file_user[index][pid_index]

    def handle_delete_timeout(self, event):
        del event
        now = rospy.Time.now()
        del_indices = []
        for n in range(0, len(self.files_list)):
            # if this time is old enough to delete:
            if (now - self.file_times[n]) > rospy.Duration(self.delete_timeout):
                # and if nobody is using it:
                if self.file_user[n] is not None:
                    # close the file
                    del_indices.append(n)
            # if this file isn't old enough to delete, we shouldn't keep
            # checking because the files get added in chronological order
            else:
                continue

        del_indices.reverse()
        for d in del_indices:
            self.files_list[d].close()
            del self.files_list[d]
            del self.files_strs[d]
            del self.file_times[d]
            del self.file_user[d]


def main():
    rospy.init_node("audiobatch_to_tempfile", anonymous=False)
    btt = BatchToTempfile()
    rospy.spin()


if __name__ == "__main__":
    main()

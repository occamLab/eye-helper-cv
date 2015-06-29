"""
generates wav files for the body mapping.
"""
import subprocess
import pydub
 

parts = ["eye", "shoulder", "elbow", "hip", "knee"]
location_shorthand = ["fa", "ha", "sa", "at", "sb", "hb", "fb"]
locations = ["a foot above", "half a foot above", "slightly above", "at", "slightly below", "half a foot below", "a foot below"]
short_to_loc = {location_shorthand[i]:locations[i] for i in range(len(location_shorthand))}

# phrases = []
# 
# for i in parts:
    # for j in locations:
        # phrases.append(j + " " + i + " level")

for i in parts:
    for j in location_shorthand:
        words = short_to_loc[j] + " " + i + " level"
        filename = "{}_{}.mp3".format(j, i[:3])
        cmd = "gtts-cli -t \"{}\" -l \"en\" {}".format(words, filename)
        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        song = pydub.AudioSegment.from_mp3(filename)
        song.export("{}_{}.wav".format(j, i[:3]), format="wav")

# for i in phrases:
    # cmd = "gtts-cli -t {} -l \"en\" {}"

# print "yay"
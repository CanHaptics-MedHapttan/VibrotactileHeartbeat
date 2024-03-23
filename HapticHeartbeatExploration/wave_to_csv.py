import csv
from scipy.io import wavfile

FILENAME = "C:\\Users\\naomi\\Documents\\GIT\\ETS\\CanHaptics\\Project\\HapticHeartbeatExploration\\audio\\hb_a_test.wav"
OUTNAME = "C:\\Users\\naomi\\Documents\\GIT\\ETS\\CanHaptics\\Project\\HapticHeartbeatExploration\\data\\samples-case-2k.csv"

samplerate, data = wavfile.read(FILENAME)
# only take one channel
channel = data #[x[0] for x in data]
with open(OUTNAME, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["samples"])
    for row in channel:
        writer.writerow([row])
import neurokit2 as nk
import numpy as np
import matplotlib.pyplot as plt

""" duration = 1  # seconds
num_samples = 1000
time = np.linspace(0, duration, num_samples) """
#signal = nk.ecg_simulate(duration=15, sampling_rate=1000, heart_rate=80)
#ecg_signal = nk.data("ecg_3000hz")
#nk.ecg_plot(ecg_signal)
#_, rpeaks = nk.ecg_peaks(signal, sampling_rate=500)
#_, waves_peak = nk.ecg_delineate(signal, rpeaks, sampling_rate=500, method="peak")


# Simulate ECG signal
ecg = nk.ecg_simulate(duration=15, sampling_rate=1000, heart_rate=80)

# Preprocess ECG signal
signals, info = nk.ecg_process(ecg, sampling_rate=1000)

# Visualize
nk.ecg_plot(signals, info)
fig = plt.gcf()
fig.set_size_inches(10, 12, forward=True) 
fig.savefig("myfig.png")
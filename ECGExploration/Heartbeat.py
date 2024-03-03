import numpy as np
import matplotlib.pyplot as plt

# Define parameters
frequency = 1  # Hz (frequency of 1 Hz corresponds to 60 beats per minute)
amplitude = 1  # normalized amplitude
duration = 1  # seconds
num_samples = 1000
time = np.linspace(0, duration, num_samples)

# Generate heartbeat waveform using sine function
#heartbeat = amplitude * np.sin(2 * np.pi * frequency * time)
heartbeat = pow(np.sin(time), 63) * np.sin(time+1.5) * 8;

# Plot the heartbeat waveform
plt.plot(time, heartbeat)
plt.title('Heartbeat Waveform')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()

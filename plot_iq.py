# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

filename = "build/rx_iq.cf32"
fs = 5e6  # 采样率，要和录制时一致

# 读取 float32
raw = np.fromfile(filename, dtype=np.float32)

# 重组成复数 IQ
iq = raw[0::2] + 1j * raw[1::2]

print("num samples =", len(iq))

# 看前 2000 个点的 I/Q 波形
n = 2000
plt.figure()
plt.plot(np.real(iq[:n]), label="I")
plt.plot(np.imag(iq[:n]), label="Q")
plt.legend()
plt.title("IQ waveform")
plt.xlabel("sample index")
plt.ylabel("amplitude")
plt.grid(True)

# 看频谱
Nfft = 4096
spec = np.fft.fftshift(np.fft.fft(iq[:Nfft]))
f = np.fft.fftshift(np.fft.fftfreq(Nfft, d=1/fs))

plt.figure()
plt.plot(f / 1e6, 20*np.log10(np.abs(spec) + 1e-12))
plt.title("Spectrum")
plt.xlabel("Frequency (MHz)")
plt.ylabel("Magnitude (dB)")
plt.grid(True)

plt.show()
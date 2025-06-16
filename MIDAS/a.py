import matplotlib.pyplot as plt
import numpy as np

# iq = np.fromfile("../../../../test/test/lora.dat", dtype="int8").astype("float").view("complex64")
iq = np.fromfile("../../../../test/test/lora.dat", dtype="int8").astype("float32").view("complex64")
print(iq)
# plt.plot(np.real(iq))
a = iq[25:]
b = iq[:-25]
pol = a * b.conj()
plt.plot(np.real(pol))
plt.plot(np.imag(pol))
plt.show()
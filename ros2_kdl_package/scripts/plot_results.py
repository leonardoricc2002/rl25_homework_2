#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ---------- CONFIGURAZIONE ----------
n_joints = 7  # Numero di giunti del robot

# ---------- Controllo argomento ----------
if len(sys.argv) < 2:
    print("Usage: python3 plot_results.py <csv_file>")
    print("Esempio: python3 plot_results.py log_vel.csv")
    sys.exit(1)

csv_file = sys.argv[1]

if not os.path.exists(csv_file):
    print(f"Errore: file '{csv_file}' non trovato!")
    sys.exit(1)

# ---------- CARICAMENTO DATI ----------
data = pd.read_csv(csv_file, header=None)

# Suddivisione posizioni e velocità
positions = data.iloc[:, :n_joints]
velocities = data.iloc[:, n_joints:]

# ---------- PLOT POSIZIONI ----------
plt.figure(figsize=(10, 6))
for i in range(n_joints):
    plt.plot(positions.iloc[:, i], label=f'joint{i+1}')
plt.title(f"Joint Positions ({csv_file})")
plt.xlabel("Step")
plt.ylabel("Position [rad]")
plt.legend(loc='upper right')
plt.grid(True)
plt.tight_layout()

# ---------- PLOT VELOCITÀ ----------
plt.figure(figsize=(10, 6))
for i in range(n_joints):
    plt.plot(velocities.iloc[:, i], label=f'joint{i+1}')
plt.title(f"Joint Velocities ({csv_file})")
plt.xlabel("Step")
plt.ylabel("Velocity [rad/s]")
plt.legend(loc='upper right')
plt.grid(True)
plt.tight_layout()

# ---------- MOSTRA ENTRAMBI I PLOT ----------
plt.show(block=True)

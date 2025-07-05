import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import detrend

# --- 1. Parámetros y Datos de Aceleración Simulados (los mismos que antes) ---
duracion_total = 4.0
dt = 0.01
n_puntos = int(duracion_total / dt)
tiempo = np.linspace(0, duracion_total, n_puntos)

# Usamos exactamente el mismo perfil de aceleración para una comparación justa
aceleracion_subida = np.sin(np.linspace(0, np.pi, int(1.5 / dt))) * 4.5
aceleracion_subida -= np.linspace(0, 5, len(aceleracion_subida))
aceleracion_bajada_controlada = np.full(int(1.5 / dt), -1.0)
aceleracion_frenada = np.linspace(-1.0, 6.0, int(1.0 / dt))
aceleracion_bajada = np.concatenate([aceleracion_bajada_controlada, aceleracion_frenada])
aceleracion_simulada = np.concatenate([aceleracion_subida, aceleracion_bajada])
if len(aceleracion_simulada) > n_puntos:
    aceleracion_simulada = aceleracion_simulada[:n_puntos]

# --- 2. Método 1: Integración Simple de Euler (para ver el problema de deriva) ---
velocidad_euler = np.zeros(n_puntos)
posicion_euler = np.zeros(n_puntos)
for i in range(1, n_puntos):
    velocidad_euler[i] = velocidad_euler[i-1] + aceleracion_simulada[i-1] * dt
    posicion_euler[i] = posicion_euler[i-1] + velocidad_euler[i-1] * dt

# --- 3. Método 2: Solucionador Avanzado `scipy.integrate.solve_ivp` (La Forma Precisa) ---

# `solve_ivp` necesita una función que describa el sistema de ecuaciones diferenciales.
# Nuestro sistema es:
# d(posicion)/dt = velocidad
# d(velocidad)/dt = aceleracion
# El vector de estado `y` será [posición, velocidad].
def modelo_movimiento(t, y, aceleracion_data, tiempo_data):
    posicion, velocidad = y
    # Como la aceleración es una serie de datos, necesitamos interpolarla
    # para encontrar el valor en cualquier instante `t` que elija el solver.
    aceleracion_t = np.interp(t, tiempo_data, aceleracion_data)
    
    dposicion_dt = velocidad
    dvelocidad_dt = aceleracion_t
    return [dposicion_dt, dvelocidad_dt]

# Condiciones iniciales: posición = 0, velocidad = 0
y0 = [0, 0]

# Rango de tiempo para la solución
t_span = [0, duracion_total]

# ¡Resolver la ecuación!
# `t_eval` nos asegura que la solución se calcula en nuestros puntos de tiempo.
# 'RK45' es el método por defecto, un excelente Runge-Kutta adaptativo.
solucion = solve_ivp(
    modelo_movimiento,
    t_span,
    y0,
    args=(aceleracion_simulada, tiempo),
    dense_output=True,
    t_eval=tiempo
)

# Extraer los resultados de la solución
posicion_scipy = solucion.y[0]
velocidad_scipy = solucion.y[1]


# --- 4. Método 3: Corrección de Deriva (Detrending) sobre el resultado de Euler ---
# Esto elimina la tendencia lineal. Es una forma de forzar el final a cero.
velocidad_detrend = detrend(velocidad_euler)
# Integramos la velocidad ya corregida para obtener una posición también corregida.
posicion_detrend = np.zeros(n_puntos)
for i in range(1, n_puntos):
    posicion_detrend[i] = posicion_detrend[i-1] + velocidad_detrend[i-1] * dt
posicion_detrend = detrend(posicion_detrend) # Aplicamos de nuevo a la posición

# --- 5. Visualización Comparativa ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
fig.suptitle('Comparación de Métodos de Integración y Corrección de Deriva', fontsize=16)

# Gráfica de Velocidad
ax1.plot(tiempo, velocidad_euler, 'r--', label='Euler Simple (con deriva)', alpha=0.8)
ax1.plot(tiempo, velocidad_scipy, 'g-', label='SciPy `solve_ivp` (Preciso)', linewidth=2)
ax1.plot(tiempo, velocidad_detrend, 'b:', label='Euler + Corrección (Detrend)', linewidth=2)
ax1.set_ylabel('Velocidad ($m/s$)')
ax1.set_title('Comparativa de Cálculo de Velocidad')
ax1.grid(True)
ax1.axhline(0, color='black', linewidth=0.5)
ax1.legend()

# Gráfica de Posición
ax2.plot(tiempo, posicion_euler, 'r--', label='Euler Simple (con deriva)', alpha=0.8)
ax2.plot(tiempo, posicion_scipy, 'g-', label='SciPy `solve_ivp` (Preciso)', linewidth=2)
ax2.plot(tiempo, posicion_detrend, 'b:', label='Euler + Corrección (Detrend)', linewidth=2)
ax2.set_ylabel('Posición (m)')
ax2.set_title('Comparativa de Cálculo de Posición (Doble Integración)')
ax2.set_xlabel('Tiempo (s)')
ax2.grid(True)
ax2.axhline(0, color='black', linewidth=0.5)
ax2.legend()

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()

# Imprimir los valores finales para ver la deriva numéricamente
print(f"VALORES FINALES EN t={duracion_total}s:")
print(f"------------------------------------")
print(f"Método Euler Simple:")
print(f"  Velocidad final: {velocidad_euler[-1]:.4f} m/s")
print(f"  Posición final:  {posicion_euler[-1]:.4f} m")
print(f"\nMétodo SciPy 'solve_ivp':")
print(f"  Velocidad final: {velocidad_scipy[-1]:.4f} m/s")
print(f"  Posición final:  {posicion_scipy[-1]:.4f} m")
print(f"\nMétodo Euler + Detrend:")
print(f"  Velocidad final: {velocidad_detrend[-1]:.4f} m/s")
print(f"  Posición final:  {posicion_detrend[-1]:.4f} m")
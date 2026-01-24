# Sistema de Seguimiento por Video para Robots Omnidireccionales

Extracción automática de trayectorias completas `[x, y, φ, vx, vy, ω]` mediante optical flow y fusión con datos de IMU.

---

## Estructura de Carpetas

```
Video_Robot Traking/
├── videos/                 # Videos de experimentos
├── Exp_videos_sen/        # Datos IMU (.txt)
├── traj_vid_data/         # Salida intermedia (video solo)
├── traj_vid_processed/    # ✓ Salida final (video + IMU)
├── track_simple_robust.py # 1. Procesamiento de video
└── process_imu_data.py    # 2. Fusión con IMU
```

---

## Uso Rápido

### **Paso 1:** Procesar Video
```bash
python track_simple_robust.py
```
- Configurar `VIDEO_NUMBER = 1` (línea 16)
- Interactivo: seleccionar robot y calibrar perspectiva (baldosa 30cm)
- Salida: `traj_vid_data/traj_vid_1.csv`

### **Paso 2:** Fusionar con IMU (Batch Automático)
```bash
python process_imu_data.py
```
- Procesa automáticamente experimentos 1-10
- **Salida final:** `traj_vid_processed/traj_vid_X.csv` (listo para identificación)

---

## Formato de Datos

### Salida Final (`traj_vid_processed/traj_vid_X.csv`)
| Columna | Descripción | Unidad |
|---------|-------------|--------|
| `time_s` | Tiempo (100 Hz, dt=0.01s) | s |
| `x_m`, `y_m` | Posición (origen en t=0) | m |
| `phi_rad` | Orientación (IMU, invertida y unwrapped) | rad |
| `vx_m_s`, `vy_m_s` | Velocidades lineales | m/s |
| `omega_rad_s` | Velocidad angular (gradiente de φ) | rad/s |

**Características clave:**
- ✓ 1000 muestras exactas (10 segundos @ 100 Hz)
- ✓ Sincronización automática: t=0 inicia 30ms antes del primer movimiento
- ✓ Orientación inicial: φ₀ ≈ π/2 ± 5%
- ✓ Sistema de coordenadas: origen en posición inicial, Y hacia arriba

---

## Métodos Implementados

### Tracking Visual (Lucas-Kanade Optical Flow)
- Grid de 100 puntos sobre el robot
- Detección de orientación: batería azul/morada (HSV: [100-150, 80-255, 30-180])
- Calibración de perspectiva con baldosa de 30×30 cm

### Procesamiento de IMU
- **Corrección de inversión:** α invertido (`-alpha`) porque CCW disminuye en sensor
- **Unwrap:** elimina discontinuidades 2π→0 **antes** de ajustar offset
- **Suavizado:** Gaussian filter (σ=3 para α, σ=2 para ω)
- **Sincronización:** detección automática de movimiento (umbral: 0.02 m/s)

### Detección de Movimiento
- Umbral de velocidad: `√(vx² + vy²) > 0.02 m/s`
- Lookback: t=0 se coloca 30ms (3 muestras) antes del primer movimiento
- Garantiza captura del transitorio inicial

---

## Visualización (Formato IEEE)

Gráficas de 4 subplots (7.16" × 5.5", 300 DPI, Times New Roman):
- **(a)** Trayectoria X-Y con vectores de orientación
- **(b)** Orientación φ(t)
- **(c)** Velocidades lineales vx, vy
- **(d)** Velocidad angular ω(t)

---

## Requisitos

```bash
pip install opencv-contrib-python numpy pandas scipy matplotlib
```

**Versión OpenCV:** 4.12.0 (bug en `tracker.init()`, por eso se usa optical flow)

---

## Notas Importantes

- **Datos para identificación:** usar solo archivos en `traj_vid_processed/`
- **Orientación φ y ω:** provienen exclusivamente de la IMU (alta precisión)
- **Posiciones x, y:** provienen del video (calibradas con perspectiva)
- **Procesamiento batch:** `process_imu_data.py` procesa automáticamente experimentos 1-10

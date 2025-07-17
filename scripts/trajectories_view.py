# === Imports ===
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
from pyproj import CRS, Transformer
from datetime import timedelta
from config import *  # Includes PATH_FILES, OUTPUT_FILE, MAP_OFFSET, DEBUG, FPS

# === Plot Settings ===
plt.rc('text', usetex=False)
plt.rc('font', family='serif')
plt.rc('font', serif='cm')
plt.rc('font', size=13)

def cartesian_to_latlon(x, y, origin_lat, origin_lon):
    """
    Converte coordenadas cartesianas locais (metros) para latitude/longitude.
    Versão corrigida para lidar com projeções UTM corretamente.
    
    Args:
        x, y: Coordenadas cartesianas em metros (relativas à origem)
        origin_lat, origin_lon: Coordenadas do ponto de origem (graus decimais)
    
    Returns:
        tuple: (latitude, longitude) em graus decimais
    """
    # Determina a zona UTM automaticamente
    utm_zone_number = int((origin_lon + 180) / 6) + 1
    utm_zone_letter = 'S' if origin_lat < 0 else 'N'
    
    # Put the UTM parameters form the correct place
    utm_crs = CRS.from_dict({
        'proj': 'utm',
        'zone': 17,
        'south': False,  # Hemisfério norte,
        'datum': 'WGS84'
    })
    
    # Sistema de coordenadas WGS84 (lat/lon)
    wgs84_crs = CRS.from_string("EPSG:4326")
    
    # Cria o transformador
    transformer = Transformer.from_crs(utm_crs, wgs84_crs, always_xy=True)
    
    # Converte a origem para UTM
    origin_transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
    origin_x, origin_y = origin_transformer.transform(origin_lon, origin_lat)
    
    # Calcula as coordenadas UTM absolutas
    utm_x = origin_x + x
    utm_y = origin_y + y
    
    # Converte de volta para WGS84 (lat/lon)
    lon, lat = transformer.transform(utm_x, utm_y)
    
    return lat, lon

# === Data Loading and Preprocessing ===
def load_and_process_gt(paths):
    gt_frame = []
    df = pd.read_csv(paths[0])

    # Remove zero coordinates
    # df = df[(df['longitude'] != 0) & (df['latitude'] != 0)]

    # Identifica índices com valores zero
    zero_lon = df['longitude'] == 0
    zero_lat = df['latitude'] == 0

    # Cria cópias deslocadas para interpolação
    lon_prev = df['longitude'].shift(1)
    lon_next = df['longitude'].shift(-1)
    lat_prev = df['latitude'].shift(1)
    lat_next = df['latitude'].shift(-1)

    # Aplica a interpolação
    df.loc[zero_lon, 'longitude'] = (lon_prev + lon_next)[zero_lon] / 2
    df.loc[zero_lat, 'latitude'] = (lat_prev + lat_next)[zero_lat] / 2

    # Combine seconds and nanoseconds into a single timestamp
    # Convert to pandas Timestamp (if not already in datetime format)
    if 'stamp_sec' in df.columns and 'stamp_nsec' in df.columns:
        # Combine seconds and nanoseconds (1e9 ns = 1 s)
        df['Time'] = pd.to_datetime(df['stamp_sec'], unit='s') + \
                    pd.to_timedelta(df['stamp_nsec'], unit='ns')
        
        # Calculate time delta from the first timestamp
        df['seconds'] = (df['Time'] - df['Time'].iloc[0]).dt.total_seconds()
    else:
        # Fallback to original HHMMSS processing if columns not found
        time_str = df['Time'].astype(str).str.zfill(6)
        df['Time'] = pd.to_timedelta(
            time_str.str[:2] + 'h' + time_str.str[2:4] + 'm' + time_str.str[4:6] + 's'
        )
        df['seconds'] = df['Time'].dt.total_seconds()

    gt_frame.append(df)
    if DEBUG:
        print(gt_frame)

    # # Normalize time across all dataframes (optional)
    # if len(data_frames) > 1:
    #     earliest_time = min(df['seconds'].min() for df in data_frames)
    #     for df in data_frames:
    #         df['seconds'] -= earliest_time
    #         if DEBUG:
    #             print('Seconds in df', df['seconds'].min(), df['seconds'].max())

    return gt_frame

# === Data Loading and Preprocessing ===
def load_and_process_path(paths, origin_lat, origin_lon):
    """
    Processa caminhos e converte coordenadas x,y para lat/long
    
    Args:
        paths: Lista de caminhos para arquivos CSV
        origin_lat: Latitude do ponto de origem (graus decimais)
        origin_lon: Longitude do ponto de origem (graus decimais)
    """
    frames = []
    df = pd.read_csv(paths[1])
    zeros = (df['id'] == 0)
    
    if zeros.any():
        last_zero_index = zeros[zeros].index[-1]
        df_cut = df.loc[last_zero_index:].copy()
    else:
        df_cut = df.copy()
    
    # Reset dada_frame index
    df_cut = df_cut.reset_index(drop=True)

    # Convert to lat/long for each point
    latitudes = []
    longitudes = []

    # To remove zero point of experience map
    offset_x = df_cut['x'][0]
    offset_y = df_cut['y'][0]

    for _, row in df_cut.iterrows():
        lat, lon = cartesian_to_latlon(
            x=row['x']-offset_x,
            y=row['y']-offset_y,
            origin_lat=origin_lat,
            origin_lon=origin_lon
        )
        latitudes.append(lat)
        longitudes.append(lon)
    
    # Criar novo DataFrame com as coordenadas convertidas
    frames.append(pd.DataFrame({
        'seconds': df_cut['id'],
        'latitude': latitudes,
        'longitude': longitudes
    }))

    path_frame = frames
    if DEBUG:
            print(path_frame)
    return path_frame

# === Interpolation Function ===
# Creates interpolated trajectories to synchronize different routes
def interpolate_paths(data_frames):
    interpolated_data = []
    max_seconds = max(len(df['latitude']) for df in data_frames)

    print(f'max_seconds = {max_seconds}')
    for df in data_frames:
        seconds = np.arange(0, max_seconds)
        interpolated_data.append(pd.DataFrame({
            'seconds': seconds,
            'longitude': np.interp(seconds, df['seconds'], df['longitude']),
            'latitude': np.interp(seconds, df['seconds'], df['latitude']),
        }))
    if DEBUG:
            print(interpolated_data)

    return interpolated_data

# === Animation Function ===
def create_animation(data_frames):
    fig = plt.figure(figsize=(8, 8))
    ax = plt.axes(projection=ccrs.PlateCarree())

    # Set map extent
    all_lons = np.concatenate([df['longitude'] for df in data_frames])
    all_lats = np.concatenate([df['latitude'] for df in data_frames])

    extent = [
        all_lons.min() - MAP_OFFSET,
        all_lons.max() + MAP_OFFSET,
        all_lats.min() - MAP_OFFSET,
        all_lats.max() + MAP_OFFSET
    ]
    ax.set_extent(extent)
    if DEBUG:
        print('Extent:', extent)

    # Add satellite imagery
    tiler = cimgt.GoogleTiles(style='satellite')
    ax.add_image(tiler, 21)

    # Prepare plots
    line_plots = [ax.plot([], [], linestyle=':', label=f'Ground Truth {i+1}')[0] for i in range(len(data_frames))]
    current_positions = [ax.scatter([], [], marker='o', label=f'Current Actual Position {i+1}') for i in range(len(data_frames))]

    # Frame update function
    def update(frame):
        for line, point, df in zip(line_plots, current_positions, data_frames):
            # Alteração 1: Usar frame+1 para incluir o ponto atual no traço
            line.set_data(df['longitude'][:frame], df['latitude'][:frame])
            
            point.set_offsets((df['longitude'][frame], df['latitude'][frame]))
        
        if DEBUG:
            print('Frame ' + str(frame) + ' updated. Latitude: ' + str(df['latitude'][frame]) +  ' Latitude: ' + str(df['longitude'][frame]))
        
        plt.legend()
        return line_plots + current_positions

    # Animate
    max_len = max(len(df['seconds']) for df in data_frames)
    ani = animation.FuncAnimation(
        fig, update, frames=max_len, repeat=False, blit=True
    )
    ani.save(OUTPUT_FILE, writer='ffmpeg', fps=FPS, savefig_kwargs={'bbox_inches': 'tight'})


# === Main Execution ===
if __name__ == "__main__":
    gt_frame = load_and_process_gt(PATH_FILES)
    origin_lat = 25.75821876525879 
    origin_lon = -80.37361907958984
    path_frame = load_and_process_path(PATH_FILES, origin_lat, origin_lon)
    # interpolated_data = interpolate_paths(data_frames)
    create_animation(path_frame)

# Plot estático completo para comparação
for df in path_frame:
    plt.plot(df['longitude'], df['latitude'], '--', alpha=0.5)
    plt.title('Trajetórias Completas (Verificação)')
    print(path_frame)
    plt.show()
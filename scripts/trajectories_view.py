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
    
    # Put the UTM parameters form the correct place
    utm_crs = CRS.from_dict({
        'proj': 'utm',
        'zone': 17,
        'south': False, 
        'datum': 'WGS84'
    })
    
    # System WGS84 (lat/lon)
    wgs84_crs = CRS.from_string("EPSG:4326")
    transformer = Transformer.from_crs(utm_crs, wgs84_crs, always_xy=True)
    
    # Convert origin to UTM
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

    df = pd.read_csv(paths[0])

    # Identify indexes with zero values
    zero_lon = df['longitude'] == 0
    zero_lat = df['latitude'] == 0

    # Create shifted copies for interpolation
    lon_prev = df['longitude'].shift(1)
    lon_next = df['longitude'].shift(-1)
    lat_prev = df['latitude'].shift(1)
    lat_next = df['latitude'].shift(-1)

    # Apply interpolation
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

    seconds = np.arange(0, len(df['seconds']-1))
    gt_frame = pd.DataFrame({
        'seconds': seconds,
        'longitude': df['longitude'],
        'latitude': df['latitude']
        
    })
    if DEBUG:
        print('GT FRAME: ')
        print(gt_frame)
    return gt_frame

# === Data Loading and Preprocessing ===
def load_and_process_path(paths, origin_lat, origin_lon):
    
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

    # To remove zero point offset of experience map
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
    
    # Crate new data frame with latitude and longitude
    path_frame = pd.DataFrame({
        'seconds': df_cut['id'],
        'longitude': longitudes,
        'latitude': latitudes
    })

    # path_frame = pd.DataFrame(frames)
    if DEBUG:
            print('PATH FRAME: ')
            print(path_frame)
    return path_frame

# === Interpolation Function ===
# Creates interpolated trajectories to synchronize different routes
def interpolate_paths(gt_frame, path_frame):
    data_frames = []
    data_frames.append(gt_frame)
    data_frames.append(path_frame)

    # Make a resample of ground truth arrays to equalize the size with exp map arrays

    if DEBUG:
        print("Data Frames: ")
        print(data_frames)

    interpolated_data = []

    min_len = min(len(df) for df in data_frames) # Exp Map Length
    max_len = max(len(df) for df in data_frames) # GPS or GT length
    min_idx = min(range(len(data_frames)), key=lambda i: len(data_frames[i]))
    max_idx = max(range(len(data_frames)), key=lambda i: len(data_frames[i]))

    # Find max and minimum lontitude values
    idx_lon_max, idx_lon_min = np.argmax(data_frames[min_idx]['longitude']), np.argmin(data_frames[min_idx]['longitude'])
    extreme_indices = np.unique([0, idx_lon_min, idx_lon_max, max_len-1])

    resampled_indices = np.round(np.linspace(0, max_len-1, min_len)).astype(int)
    final_indices = np.unique(np.concatenate([extreme_indices, resampled_indices]))

    # Resample the vector while preserving extremes
    longitude_old = data_frames[max_idx]['longitude']
    longitude_resampled = longitude_old[final_indices[:min_len]] # Adjust the length to new_len
    # print(longitude_resampled)

    # latitude vector
    idx_lat_max, idx_lat_min = np.argmax(data_frames[min_idx]['latitude']), np.argmin(data_frames[min_idx]['latitude'])
    extreme_indices = np.unique([0, idx_lat_min, idx_lat_max, max_len-1])
    resampled_indices = np.round(np.linspace(0, max_len-1, min_len)).astype(int)
    final_indices = np.unique(np.concatenate([extreme_indices, resampled_indices]))
    latitude_old = data_frames[max_idx]['latitude']
    latitude_resampled = latitude_old[final_indices[:min_len]] # Adjust the length to new_len
    # print(latitude_resampled)

    resampled_frame = pd.DataFrame({
        'seconds': data_frames[min_idx]['seconds'],
        'longitude': longitude_resampled.to_numpy().copy(),
        'latitude': latitude_resampled.to_numpy().copy()
        
    })

    interpolated_data.append(resampled_frame) # Ground Truth
    interpolated_data.append(data_frames[min_idx]) # Experience map

    if DEBUG:
        print("Interpolated Data Frames: ")
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
    line_plots = [ax.plot([], [], linestyle=':')[0] for i in range(len(data_frames))]
    current_positions = [ax.scatter([], [], marker='o') for i in range(len(data_frames))]


    # Frame update function
    def update(frame):
        for line, point, df in zip(line_plots, current_positions, data_frames):
            # Alteração 1: Usar frame+1 para incluir o ponto atual no traço
            line.set_data(df['longitude'][:frame], df['latitude'][:frame])
            
            point.set_offsets((df['longitude'][frame], df['latitude'][frame]))
        
        if DEBUG:
            print('Frame ' + str(frame) + ' updated. Latitude: ' + str(df['latitude'][frame]) +  ' Latitude: ' + str(df['longitude'][frame]))
        
        plt.legend(['Ground truth', 'Estimated trajectory'], loc='upper right', frameon=True)
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
    origin_lat = gt_frame['latitude'].iloc[0] # 25.75821876525879 
    origin_lon = gt_frame['longitude'].iloc[0] # -80.37361907958984
    path_frame = load_and_process_path(PATH_FILES, origin_lat, origin_lon)
    interpolated_data = interpolate_paths(gt_frame, path_frame)
    create_animation(interpolated_data)

# Static Full plot
# for df in interpolated_data:
#     plt.plot(df['longitude'], df['latitude'], '--', alpha=0.5)

# plt.title('Trajetórias Completas (Verificação)')
# plt.legend(['Ground truth', 'Estimated trajectory'], loc='upper right', frameon=True)
# plt.show()
import osmnx as ox
import matplotlib.pyplot as plt
import gpxpy
import gpxpy.gpx
from shapely.geometry import Point, LineString
from shapely.ops import transform
import pyproj
from tqdm import tqdm

# Настройки отображения
ox.config(use_cache=True, log_console=False)

# Шаг 1: Загрузка дорожной сети всего города
def load_road_network(place_name):
    print(f"Загрузка дорожной сети для {place_name}...")
    G = ox.graph_from_place(place_name, network_type='drive')
    G = ox.project_graph(G)
    return G

# Шаг 2: Чтение .gpx файла
def load_gps_track(gpx_file):
    print(f"Чтение GPS-трека из файла {gpx_file}...")
    with open(gpx_file, 'r', encoding='utf-8') as f:
        gpx = gpxpy.parse(f)
    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for point in segment.points:
                points.append((point.longitude, point.latitude))
    return points

# Шаг 3: Проецирование GPS-точек в систему координат графа
def project_gps_points(gps_points, G):
    print("Проецирование GPS-точек в систему координат графа...")
    graph_crs = G.graph['crs']
    # Создаем трансформер для преобразования координат
    transformer = pyproj.Transformer.from_crs("epsg:4326", graph_crs, always_xy=True)
    # Проецируем точки
    projected_points = [transform(transformer.transform, Point(lon, lat)) for lon, lat in gps_points]
    return projected_points

# Шаг 4: Функция сопоставления GPS-точек с дорожной сетью
def map_match(G, gps_points):
    print("Сопоставление GPS-точек с дорожной сетью...")
    matched_points = []
    for point in tqdm(gps_points):
        try:
            # Получаем координаты x, y
            x, y = point.x, point.y
            # Поиск ближайшего ребра
            u, v, key = ox.distance.nearest_edges(G, x, y)
            edge_data = G.get_edge_data(u, v, key)
            # Получение геометрии ребра
            if 'geometry' in edge_data:
                line = edge_data['geometry']
            else:
                point_u = Point((G.nodes[u]['x'], G.nodes[u]['y']))
                point_v = Point((G.nodes[v]['x'], G.nodes[v]['y']))
                line = LineString([point_u, point_v])
            # Проекция точки на ребро
            projected_point = line.interpolate(line.project(point))
            matched_points.append(projected_point)
        except Exception as e:
            # Если не удалось сопоставить точку, добавляем оригинальные координаты
            matched_points.append(point)
    return matched_points

# Шаг 5: Визуализация результатов
def plot_results(G, gps_points, matched_points):
    print("Построение графиков...")

    # Отображение дорожной сети
    fig, ax = plt.subplots(figsize=(10, 10))
    ox.plot_graph(G, ax=ax, show=False, close=False, node_size=0, edge_color='gray')

    ax.set_title("1. Дорожная сеть города Воронеж")
    plt.show()

    # Отображение GPS-трека поверх дорожной сети
    fig, ax = plt.subplots(figsize=(10, 10))
    ox.plot_graph(G, ax=ax, show=False, close=False, node_size=0, edge_color='gray')

    gps_x = [point.x for point in gps_points]
    gps_y = [point.y for point in gps_points]

    ax.plot(gps_x, gps_y, 'r-', linewidth=2, label='GPS-трек')

    ax.set_title("2. GPS-трек поверх дорожной сети")
    ax.legend()
    plt.show()

    # Отображение сопоставленного трека
    fig, ax = plt.subplots(figsize=(10, 10))
    ox.plot_graph(G, ax=ax, show=False, close=False, node_size=0, edge_color='gray')

    ax.plot(gps_x, gps_y, 'r--', linewidth=1, label='Исходный GPS-трек')

    matched_x = [point.x for point in matched_points]
    matched_y = [point.y for point in matched_points]

    ax.plot(matched_x, matched_y, 'g-', linewidth=2, label='Сопоставленный трек')

    ax.set_title("3. Сопоставленный GPS-трек")
    ax.legend()
    plt.show()

def main():
    place_name = "Воронеж, Россия"  # Загрузка всей дорожной сети города
    gpx_file = "voronezh.gpx"

    # Шаг 1: Загрузка дорожной сети
    G = load_road_network(place_name)

    # Шаг 2: Чтение GPS-трека
    gps_points = load_gps_track(gpx_file)

    # Шаг 3: Проецирование GPS-точек
    projected_gps_points = project_gps_points(gps_points, G)

    # Шаг 4: Сопоставление GPS-точек с дорожной сетью
    matched_points = map_match(G, projected_gps_points)

    # Шаг 5: Визуализация результатов
    plot_results(G, projected_gps_points, matched_points)

if __name__ == "__main__":
    main()

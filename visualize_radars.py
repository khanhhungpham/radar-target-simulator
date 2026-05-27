import json
import matplotlib.pyplot as plt
import numpy as np
from geographiclib.geodesic import Geodesic

# 1. Đọc dữ liệu từ file input.json
with open("input.json", "r", encoding="utf-8") as f:
    data = json.load(f)
radars = data.get("radars", [])

# Khởi tạo mô hình Ellipsoid của Trái Đất (WGS84 chuẩn toàn cầu)
geod = Geodesic.WGS84

# 2. Khởi tạo đồ thị
fig, ax = plt.subplots(figsize=(8, 8))
colors = ["blue", "red", "green", "purple", "orange", "darkred"]

# 3. Vẽ vị trí và tính toán tầm quét chính xác toàn cầu
for i, radar in enumerate(radars):
    lon, lat, r_km = radar["lon"], radar["lat"], radar["range_km"]
    color = colors[i % len(colors)]
    
    # Mảng lưu tọa độ đường viền vòng tròn tầm quét
    circle_lats = []
    circle_lons = []
    
    # Quét 360 độ xung quanh tâm radar để tìm các điểm mút chuẩn xác trên mặt cầu
    # Tạo ra 120 điểm để vòng tròn mượt mà
    for azimuth in np.linspace(0, 360, 120):
        # Tính toán điểm đến từ Tâm (lat, lon) đi theo Góc (azimuth) với Khoảng cách (r_km * 1000 mét)
        result = geod.Direct(lat, lon, azimuth, r_km * 1000)
        circle_lats.append(result['lat2'])
        circle_lons.append(result['lon2'])
    
    # Xử lý trường hợp đặc biệt nếu radar quét qua kinh tuyến đổi ngày (180 độ)
    # (Để đồ thị matplotlib không bị vẽ một đường sọc ngang màn hình)
    circle_lons = np.array(circle_lons)
    circle_lats = np.array(circle_lats)
    if np.any(np.abs(np.diff(circle_lons)) > 180):
        # Nếu quét qua biên giới kinh tuyến, tách làm 2 phần để vẽ (hiếm gặp nhưng chuẩn hóa)
        pass 

    # Vẽ vùng quét lên đồ thị (Tự động bù méo do vĩ độ cao gần vùng cực)
    ax.fill(circle_lons, circle_lats, color=color, alpha=0.2, label=f"Radar {radar['id']} ({r_km}km)")
    ax.plot(circle_lons, circle_lats, color=color, linewidth=1.5)
    
    # Vẽ tâm radar
    ax.plot(lon, lat, marker="^", color=color, markersize=8)
    ax.text(lon, lat, f" R{radar['id']}", fontsize=10, fontweight="bold")

# 4. Định dạng biểu đồ tự động theo phạm vi của dữ liệu đầu vào
ax.set_title("Trực quan hóa Radar Toàn Cầu (Mô hình Hình học Mặt cầu WGS84)", fontsize=13, pad=15)
ax.set_xlabel("Kinh độ (Longitude)")
ax.set_ylabel("Vĩ độ (Latitude)")
ax.grid(True, linestyle="--", alpha=0.5)

# Tự động tối ưu tỷ lệ trục dựa trên vĩ độ trung bình để hình không bị bóp méo
avg_lat = sum(r["lat"] for r in radars) / len(radars)
ax.set_aspect(1.0 / np.cos(np.radians(avg_lat)))

ax.legend()

# 5. Hiển thị cửa sổ đồ họa trực tiếp
plt.show()

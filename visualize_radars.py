import os
import sys
import json
import matplotlib.pyplot as plt
import numpy as np
from geographiclib.geodesic import Geodesic

# 1. Đọc dữ liệu từ file input.json
if not os.path.exists("input.json"):
    print("LỖI: Không tìm thấy file 'input.json'.")
    sys.exit(1)

with open("input.json", "r", encoding="utf-8") as f:
    try:
        data = json.load(f)
    except json.JSONDecodeError:
        print("LỖI: Định dạng file 'input.json' không hợp lệ.")
        sys.exit(1)

radars = data.get("radars", [])
if not radars:
    print("CẢNH BÁO: Không có dữ liệu radar nào trong file cấu hình.")
    sys.exit(0)

# Khởi tạo mô hình Ellipsoid của Trái Đất (WGS84 chuẩn toàn cầu)
geod = Geodesic.WGS84

# 2. Khởi tạo đồ thị
fig, ax = plt.subplots(figsize=(9, 9))
colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

# 3. Vẽ vị trí và tính toán tầm quét chính xác toàn cầu
for i, radar in enumerate(radars):
    lon, lat, r_km = radar["lon"], radar["lat"], radar["range_km"]
    color = colors[i % len(colors)]
    
    circle_lats = []
    circle_lons = []
    
    # Quét 360 độ tạo ra 120 điểm mút chính xác mặt cầu
    for azimuth in np.linspace(0, 360, 120):
        result = geod.Direct(lat, lon, azimuth, r_km * 1000)
        circle_lats.append(result['lat2'])
        circle_lons.append(result['lon2'])
    
    circle_lons = np.array(circle_lons)
    circle_lats = np.array(circle_lats)

    # Vẽ vùng quét đổ màu (Gán label ở đây để tránh trùng lặp trong Legend)
    ax.fill(circle_lons, circle_lats, color=color, alpha=0.15, label=f"Radar {radar['id']} ({r_km}km)")
    # Vẽ đường viền
    ax.plot(circle_lons, circle_lats, color=color, linewidth=1.2, linestyle="-")
    
    # Vẽ tâm radar (Dùng ký tự hình tam giác đại diện cho trạm radar)
    ax.plot(lon, lat, marker="^", color=color, markersize=8, markeredgecolor='black', markeredgewidth=0.5)
    ax.text(lon, lat, f" R{radar['id']}", fontsize=10, fontweight="bold", verticalalignment='bottom')

# 4. Định dạng biểu đồ tự động
ax.set_title("Trực quan hóa vùng phủ radar (Mô hình Trái Đất WGS84)", fontsize=13, pad=15, fontweight="bold")
ax.set_xlabel("Kinh độ (Longitude - Degree)")
ax.set_ylabel("Vĩ độ (Latitude - Degree)")
ax.grid(True, linestyle="--", alpha=0.5)

# Tính toán tỷ lệ trục an toàn (Tránh lỗi chia cho 0)
avg_lat = sum(r["lat"] for r in radars) / len(radars)
# Giới hạn góc để hàm cos không tiến về 0 nếu radar đặt ở cực
avg_lat = max(-85.0, min(85.0, avg_lat)) 
ax.set_aspect(1.0 / np.cos(np.radians(avg_lat)))

# Hiển thị chú thích góc trên bên phải gọn gàng
ax.legend(loc="upper right", framealpha=0.9)

# Thắt chặt layout chống mất chữ ở rìa
plt.tight_layout()

# 5. Hiển thị cửa sổ đồ họa
plt.show()

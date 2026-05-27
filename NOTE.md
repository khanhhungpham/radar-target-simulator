# Prompt

## Simulate radar targets

Viết chương trình simulate_radar_targets.py giả lập như sau (bài toán tập trung sinh dữ liệu cho hợp nhất mục tiêu):

### Đầu vào của chương trình là file input.json chứa:

Vị trí của các radar (lat, lon), bán kính quét theo km, chu kỳ quét theo giây

Số lượng mục tiêu

### Chương trình

Sinh ra các mục tiêu nằm trong vùng của các radar, các mục tiêu này sẽ di chuyển tự do với vận tốc dưới 30 hải lý/giờ

### Đầu ra của chương trình là outputs/<YYYYMMDD_hhmmss>/radar_target_data.csv gồm:

Timestamp (datetime, đơn vị là giây), id mục tiêu (độc lập với từng radar), id mục tiêu ground truth, cự ly, phương vị, vận tốc, hướng mục tiêu, lat, lon

## Visualize radars



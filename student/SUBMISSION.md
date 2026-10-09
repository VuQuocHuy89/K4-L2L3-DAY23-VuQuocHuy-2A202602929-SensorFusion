# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Vu Quoc Huy
- MSSV: 2A202602929
- Email: vuquochuyforwork@gmail.com
- Link repo (fork): https://github.com/VuQuocHuy89/K4-L2L3-DAY23-VuQuocHuy-2A202602929-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): f2bc97285b78d126fc5c0dd1961dd567b2699823

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: compare; [0, 198]; training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord; 0.
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: 0.9701; 0.7004; 519/16/222.
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1503 m; 502; 11.3436 m²; 0; 239; 2.5226.
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1359 m; 502; 9.2668 m²; 0; 239; 2.5226.
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: Fused giảm RMSE 0.0145 m; cả hai mode có 502 matches, 0 ghost và 239 missed GT. Detection giống nhau giữa hai mode.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)? Lidar đo XYZ với `z` 3 chiều và `R` theo m²; camera đo pixel (u, v) với `z` 2 chiều, `R` theo pixel² và hàm chiếu pinhole phi tuyến.
2. Vì sao cần gating Mahalanobis trước khi gán? Khoảng cách Mahalanobis chuẩn hóa innovation theo bất định `S`; cổng χ² loại cặp không phù hợp với số chiều đo và độ bất định của track.
3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`. Track-then-fuse: mỗi frame predict một lần, cập nhật LiDAR rồi camera trên cùng track; `grade_run.log` có record cho cả hai mode theo từng frame.
4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual? Sai lệch tạo innovation/residual camera có bias; khi vượt cổng χ² phép gán bị loại, còn nếu lọt cổng thì update có thể kéo lệch state và tăng RMSE.
5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng? Sensor cho manager biết lượt xử lý là LiDAR hay camera dù không có measurement; lượt LiDAR vẫn tính miss và quản lý track, còn camera không đổi vòng đời.
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   LiDAR quyết định score, khởi tạo và xóa track; camera chỉ cập nhật EKF, không tác động score hoặc vòng đời.
6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track. Score tăng `1/window` mỗi hit (tối đa 1), xác nhận khi score > 0.8; confirmed vẫn confirmed sau miss. Xóa nếu `P[0,0]` hoặc `P[1,1]` > 9, confirmed có score < 0.6, hoặc track chưa confirmed có score ≤ 0.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): ChatGPT (Codex).
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ triển khai và debug Parts E–H, chuẩn bị môi trường, tổng hợp giải thích.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` (128 passed); chạy `fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0` trên frame 0–198; kiểm tra metrics và invariant trong log.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))

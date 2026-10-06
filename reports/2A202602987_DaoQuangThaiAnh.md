# Báo cáo cá nhân T4: Benchmark độ trễ radar-IMU và đánh giá EKF-RIO-TC

**Người viết:** Đào Quang Thái Anh (MSSV 2A202602987)  
**Bài thực hành:** Track 4, Ngày 4 (Sensor Reality Sprint), chủ đề T4: Time sync / motion compensation  
**Vai trò trong nhóm:** phụ trách benchmark và trình bày: thiết kế điều kiện lỗi và metric, đọc và kiểm tra bảng số trong `results/`, chọn hình và dựng phần tóm tắt (pitch) của báo cáo nhóm. Phân công của cả nhóm ở [`TEAMMATES.md`](../TEAMMATES.md).  
**Bằng chứng chung của nhóm:** [báo cáo nhóm](../README.md), thư mục [`results/`](../results/) (bảng số, log, hình), code ở commit [`1afade6`](https://github.com/vvstdung89/K4-Track4-Day04-Studio.h-Sensor-Reality-Sprint/commit/1afade6).

Quy ước: "Bài báo cho biết" là kết luận của tác giả bài báo; "Nhóm đo được" là kết quả nhóm tự chạy trong mô phỏng; các điểm chưa đo trực tiếp được ghi rõ là suy luận.

## 1. Problem

Trong xe có ADAS, radar 4D và IMU cùng ước lượng **vận tốc ego** và đặt mục tiêu radar vào bản đồ quanh xe. Hai đầu ra này đi thẳng vào AEB, ACC và radar tracking. Lỗi thực tế nhóm chọn là **radar lệch thời gian so với IMU**: radar phải xử lý xong cả khung (FFT, beamforming, phát hiện mục tiêu) rồi mới gửi đi, và nếu driver gán timestamp lúc nhận thì toàn bộ thời gian này trở thành độ trễ. Bài báo cho biết với radar TI AWR1843BOOST, độ trễ này khoảng 113 ms, và cả dữ liệu tự thu lẫn ColoRadar đều không có đồng bộ phần cứng radar-IMU.

Câu hỏi tôi phụ trách trả lời bằng benchmark là: **độ trễ bao nhiêu thì bắt đầu nguy hiểm, nguy hiểm ở metric nào, và metric nào có thể "che giấu" lỗi.**

**Claim của nhóm.** Khi radar trễ 50-200 ms, sai số vị trí mục tiêu ≈ *tốc độ × độ trễ* và sai số vận tốc ego ≈ *gia tốc × độ trễ*; cả hai tăng theo độ trễ. Bù trễ online theo bài báo đưa các sai số này về gần mức không trễ, với điều kiện xe có đủ chuyển động.

## 2. Method

**Bài báo:** Kim et al., *EKF-Based Radar-Inertial Odometry with Online Temporal Calibration*, IEEE RA-L 2025, [arXiv 2502.00661v2](https://arxiv.org/abs/2502.00661v2) (v2, 10/06/2025). Code gốc: [spearwin/EKF-RIO-TC](https://github.com/spearwin/EKF-RIO-TC) (C++/ROS Noetic). Nhóm không chạy code gốc (cần Ubuntu 20.04, catkin, rosbag; không khả thi trong 120 phút trên máy Windows), mà cài lại bản 2-D bằng Python trong [`rio_tc_sim.py`](../rio_tc_sim.py).

**Ý tưởng.** Độ trễ `t_d` được đưa vào trạng thái của EKF và ước lượng cùng hướng, vận tốc, bias IMU. Mỗi scan radar, bộ lọc tích phân IMU tới `t + t̂_d`, dự đoán vận tốc radar lẽ ra phải đo, so với vận tốc đo thật, rồi sửa toàn bộ trạng thái kể cả `t_d`. Độ nhạy của residual theo `t_d` chính là tốc độ thay đổi của đại lượng đo (gia tốc và vận tốc góc), nên xe càng tăng tốc/quay mạnh thì `t_d` càng dễ quan sát.

| | |
| --- | --- |
| **Input** | IMU (gia tốc, vận tốc góc; 200 Hz trong mô phỏng) và vận tốc ego radar từ mỗi scan (15 Hz) |
| **Output** | `t̂_d` kèm σ_td, vận tốc ego, quỹ đạo |
| **Giả định** | `t_d` gần như hằng số; điểm động/điểm ma đã được RANSAC loại; xe có đủ kích thích |
| **Metric của bài báo** | offset error (chèn trễ biết trước vào dữ liệu đã đồng bộ), APE, RPE 10 m; baseline EKF-RIO không bù trễ |

**Limitation.** Bài báo cho biết `t_d` không xác định được khi đứng yên, và có đánh đổi giữa hội tụ nhanh và ổn định khi chọn nhiễu quá trình. Benchmark của nhóm đo được thêm:

- **Ngưỡng kích thích:** `t_d` chỉ hội tụ 10/10 lần khi biên độ gia tốc ≥ 1 m/s²; ở 0,5 m/s² chỉ 1/10 ([`results.md`](../results/results.md), E3).
- **Sai mà tự tin:** ở gia tốc 0, `t̂_d` trôi 318 ms trong khi σ_td báo 10,6 ms. Chạy đều 40 s ở 25 m/s rồi phanh thì sai 320 ms lúc phanh (σ 6,7 ms) và vận tốc ego sai 1,03 m/s, tệ hơn không bù (0,64 m/s) (E5).
- **Không theo kịp trễ thay đổi:** trễ tăng 50 ms giữa chuyến thì không về lại < 10 ms ở cả 10 seed; tăng nhiễu quá trình 50 lần thì bám sau 5,6 s nhưng dao động gấp 5,7 lần (E4).

## 3. Benchmark

Đây là phần tôi chịu trách nhiệm chính. Nguyên tắc thiết kế: **mỗi điều kiện lỗi chỉ đổi một tham số (độ trễ), mọi thứ khác giữ nguyên**, và luôn có cả cận dưới (không bù) lẫn cận trên (oracle) để đọc được phương pháp nằm ở đâu.

**Dữ liệu.** Hoàn toàn tổng hợp, sinh trong `rio_tc_sim.py`, không dùng dataset ngoài. Xe 2-D không trượt ngang; IMU 200 Hz (nhiễu 0,10 m/s², 0,005 rad/s, có bias); radar 15 Hz (nhiễu 0,05 m/s, gắn cách IMU 3,5 m). Hai kiểu lái: **phố** 60 s (±2,4 m/s², yaw rate ±0,25 rad/s) và **cao tốc** 25 m/s chạy đều 40 s rồi phanh −6 m/s². Mỗi số là trung bình 10 seed (0-9), riêng ablation E6 là 20 seed.

**Baseline và điều kiện lỗi.**

| Điều kiện | Giá trị |
| --- | --- |
| Baseline | `t_d` = 0 (không lệch thời gian) |
| Điều kiện lỗi | `t_d` ∈ {50, 113, 150, 200} ms (thêm −50 ms… +50 ms để kiểm dấu) |
| Cách xử lý so sánh | không bù (timestamp lúc nhận) · phương pháp bài báo (TC) · oracle (biết `t_d` thật) |

**Metric (càng nhỏ càng tốt).** Tôi map 3 metric yêu cầu của T4 sang đại lượng đo được trong mô phỏng:

| Metric T4 | Cài đặt | Đơn vị | Vì sao chọn |
| --- | --- | --- | --- |
| Offset error | \|`t̂_d` − `t_d`\|; thời gian hội tụ = từ lúc sai < 10 ms mãi về sau | ms, s | đúng giao thức chèn trễ của bài báo |
| Ghost rate (proxy) | tỉ lệ scan có NIS > 9,21 (χ²₂ 99 %), tức scan một bộ gating sẽ bỏ như "ma" | % | đo được mà không cần mô phỏng tracker |
| Trajectory residual | RMSE vận tốc ego; RPE 10 m; APE sau căn SE(2) | m/s, m | vận tốc cho AEB/ACC, RPE để so với bài báo |
| Sai số vị trí mục tiêu | mục tiêu tĩnh cách 30 m, đặt bằng pose *thật* tại thời điểm hệ thống *nghĩ* là lúc đo | m | benchmark tối thiểu của T4; cô lập riêng lỗi thời gian |

**Proxy chưa chứng minh được gì.** Gate-fail rate không phải số mục tiêu ma thật trong tracker. Sai số vị trí dùng pose thật nên chưa gồm sai số odometry. Benchmark không đo trực tiếp AEB hay tracker thật.

**Lệnh chạy.** `python rio_tc_sim.py` (~8 phút, log ghi 458 s) → `python position_error.py` → `python export_failure_log.py` → `python plot_delay_impact.py` → `python check_claims.py`. Môi trường: Python 3.10.11, numpy 2.2.6, matplotlib 3.10.9. Kết quả: [`results/results.md`](../results/results.md), [`results/position_error.md`](../results/position_error.md), log [`results/run_log.txt`](../results/run_log.txt).

**Kiểm tra trước khi tin số.** Trước khi đưa số vào báo cáo, tôi đối chiếu ba điểm:

- Jacobian kiểm bằng sai phân hữu hạn: sai khác ≤ 9,9e-10, cột `t_d` ≤ 5,3e-11 ([`check_claims.txt`](../results/check_claims.txt)).
- Trên đường thẳng tốc độ đều, sai số vị trí đo được **khớp đúng** công thức tốc độ × độ trễ ở mọi ô 5-30 m/s × 50-200 ms (ví dụ 30 m/s, 200 ms: 6,000 m). Metric vị trí vì vậy đo đúng thứ nó định đo.
- Sai số `t_d` còn lại +2,8 ms là do tích phân IMU giữ mẫu (nửa chu kỳ 200 Hz = 2,5 ms): tăng IMU lên 400 Hz thì còn +1,4 ms. Đây là giới hạn của mô phỏng, không phải của phương pháp.

### 3.1 Kết quả chính — lái trong phố

![Timeline và sai số vị trí](../results/fig6_timeline_position_error.png)

*Hình 6: timeline IMU/radar (radar đo lúc xanh lá, gán timestamp muộn 113 ms lúc đỏ, sau bù khớp lúc xanh dương); sai số vị trí trên đường thẳng; sai số vị trí trong phố trước/sau khi bù.*

| Điều kiện | Xử lý | Vị trí mục tiêu | Vận tốc ego | Gate-fail | RPE 10 m | APE |
| --- | --- | --- | --- | --- | --- | --- |
| Baseline (0 ms) | không bù | 0 m | 0,018 m/s | 1,2 % | 0,044 m | 1,67 m |
| 50 ms | không bù | 0,58 m | 0,070 m/s | 7,7 % | 0,102 m | 2,79 m |
| **113 ms** | **không bù** | **1,31 m** | **0,154 m/s** | **46,0 %** | **0,196 m** | **4,22 m** |
| 150 ms | không bù | 1,74 m | 0,204 m/s | 61,2 % | 0,251 m | 5,11 m |
| 200 ms | không bù | 2,31 m | 0,271 m/s | 72,7 % | 0,324 m | 7,25 m |
| **113 ms** | **TC (bài báo)** | **0,03 m** | **0,025 m/s** | **1,3 %** | **0,047 m** | **1,81 m** |
| 113 ms | oracle | 0 m | 0,018 m/s | 1,2 % | 0,044 m | 1,67 m |

**Đọc bảng.**

- Mọi metric xấu dần gần tuyến tính theo độ trễ. Gate-fail tăng nhanh nhất: chỉ từ 50 lên 113 ms đã nhảy từ 7,7 % lên 46 %, tức gần nửa dữ liệu radar bị bỏ.
- Khi có đủ chuyển động, TC đưa mọi metric về sát oracle; offset error 2,8 ms, hội tụ sau 5,7 s; hội tụ về cùng giá trị từ khởi tạo 0, −250 hay +100 ms (E2).
- Thêm `t_d` vào trạng thái là an toàn khi trễ thật bằng 0 (RPE 0,046 so với 0,044 m).
- Trong phố, sai số vị trí lớn hơn công thức khoảng 13 % (1,31 so với 1,16 m) vì lúc xe rẽ mục tiêu còn bị xoay thêm.
- Nhóm đo được RPE giảm 76 %, APE giảm 57 %; bài báo cho biết APE giảm 56 %, RPE giảm 50 % trên dữ liệu thật. Hai bên khác dữ liệu nên chỉ so xu hướng, không so trực tiếp.

**Giới hạn của benchmark.** Chỉ 2-D, không trọng lực; không mô phỏng từng điểm Doppler và RANSAC; nhiễu Gauss lý tưởng, không rung, không sai ngoại tham số; chưa chạy code gốc trên dữ liệu thật; chưa làm phần mở rộng rolling-shutter.

## 4. Failure case: metric "khỏe" nhưng hệ thống đang sai

Từ góc nhìn benchmark, failure tôi thấy đáng lo nhất **không phải lúc sai số lớn, mà lúc sai số lớn mà không metric giám sát nào báo động**.

**Kịch bản (E5, E7).** Xe chạy cao tốc 25 m/s thẳng đều 40 s, rồi AEB phanh −6 m/s² xuống 5 m/s. `t_d` thật = 113 ms và **không được bù**, đúng như một pipeline chưa xử lý vấn đề này.

![Ảnh hưởng của trễ radar tới vận tốc ego](../results/fig5_delay_impact.png)

*Hình 5: trái/giữa là E1 (phố, 10 seed); phải là E5 (một lần chạy, seed 1). Log từng scan: [`e5_failure_log_seed1.csv`](../results/e5_failure_log_seed1.csv), bản trích [`e5_failure_log_excerpt.txt`](../results/e5_failure_log_excerpt.txt).*

**Nhóm đo được, theo từng pha:**

| Pha | Vị trí mục tiêu (không bù) | Vận tốc ego (không bù) | Gate-fail | Metric giám sát có báo không? |
| --- | --- | --- | --- | --- |
| Chạy đều 5-40 s | **2,83 m** RMSE | 0,018 m/s (= oracle) | ~1 % | **Không.** Vận tốc và NIS đều bình thường |
| Phanh 40-43,3 s | 1,78 m RMSE | **0,64 m/s** RMSE, đỉnh 0,73 m/s | **13 %** | Rất yếu; seed 1 chỉ 10 % scan bị loại |
| Sau phanh (44 s) | – | 0,41 m/s, còn kéo dài ~3 s | NIS 66,6 | Chỉ báo **sau khi** pha nguy hiểm đã qua |

- Sai số vị trí 2,8 m khi chạy đều lớn hơn nửa làn 1,75 m, nhưng vận tốc ego lại bằng đúng oracle. Ai chỉ theo dõi vận tốc ego hay NIS sẽ kết luận hệ thống khỏe.
- Khi phanh, sai số 0,64-0,73 m/s khớp ước tính 6 m/s² × 0,113 s ≈ 0,68 m/s. Hệ thống luôn báo xe **nhanh hơn thực tế** vì dùng vận tốc của 113 ms trước.
- Gating không bắt được vì bộ lọc đã tự chỉnh vận tốc khớp với phép đo bị trễ: chênh lệch đo/dự đoán nhỏ dù cả hai đều sai. Trong log seed 1, NIS chỉ vượt 9,21 ở 44 s, khi xe đã phanh xong.
- Ngược lại, trong phố gate-fail lên 46 %: cùng một lỗi nhưng biểu hiện ra metric hoàn toàn khác tùy kiểu lái. Một ngưỡng gate-fail cố định không dùng làm cảnh báo trễ được.

**Ngưỡng nguy hiểm** (nhóm tự chọn: 0,5 m ≈ cổng ghép mục tiêu giả định, 1,75 m = nửa làn 3,5 m), từ sai số = tốc độ × độ trễ:

| Tốc độ | Trễ tối đa để sai < 0,5 m | Trễ tối đa để sai < 1,75 m |
| --- | --- | --- |
| 10 m/s | 50 ms | 175 ms |
| 20 m/s | 25 ms | 88 ms |
| 25 m/s | 20 ms | 70 ms |
| 30 m/s | 17 ms | 58 ms |

Trễ 113 ms vượt cả hai ngưỡng từ 20 m/s trở lên. Muốn an toàn ở cao tốc, sai lệch thời gian còn lại sau bù phải < ~20 ms.

**Bài báo cho biết** trên 7 chuỗi thật, bỏ qua trễ làm APE tăng từ 0,81 m / 2,4° lên 1,82 m / 9,7°.

**Ảnh hưởng tới tính năng (suy luận, chưa đo).** AEB/ACC nhận vận tốc ego cao hơn thật khoảng 0,7 m/s đúng lúc phanh khẩn cấp; radar tracking trừ vận tốc ego khỏi Doppler nên vật tĩnh có thể bị gán vận tốc khoảng 0,7 m/s. Ở 25 m/s, xe làn bên có thể bị gán nhầm vào làn mình.

**Khi bật phương pháp bài báo cũng không cứu được kịch bản này:** sau 40 s chạy đều, `t̂_d` đã trôi, vị trí mục tiêu sai 8,4 m ở giây cuối (khởi tạo đúng: 6,1 m) và vận tốc ego khi phanh sai 1,03 m/s ([`position_error.md`](../results/position_error.md) P3; E5). Ablation 20 seed cho thấy nguyên nhân là `H_td` tính từ IMU thô: tắt nhiễu gyro giảm độ trôi từ 313 xuống 107 ms.

## 5. Engineering decision

**Quyết định:** bắt buộc bù trễ radar, nhưng **không tin σ_td hay NIS của bộ lọc làm bằng chứng là đã đồng bộ**. Phần 4 cho thấy cả hai có thể "xanh" trong khi vị trí sai 2,8 m hoặc vận tốc sai 0,7 m/s.

| Tình huống | Quyết định | Căn cứ (nhóm đo được) |
| --- | --- | --- |
| Phố, gia tốc thường xuyên ≥ 1 m/s² | Dùng ước lượng online | Hội tụ 3-13 s; vị trí 1,31 → 0,03 m; gate-fail 46 % → 1,3 % |
| Cao tốc chạy đều lâu | Khóa `t_d` ở giá trị offline, không cho học | TC sau 40 s chạy đều: vị trí 8,4 m, vận tốc lúc phanh 1,03 m/s |
| Trễ đã đo offline, ổn định | Dùng giá trị cố định, chỉ giám sát | Bù đúng `t_d` cho kết quả bằng oracle |
| Trễ đổi theo tải CPU / chế độ radar | Cần phát hiện thay đổi riêng | Trễ tăng 50 ms: TC không bám được trong 45 s |

**Đề xuất (chưa cài đặt):**

1. **Mặc định bù bằng `t_d` hiệu chỉnh offline** cho từng loại radar; ước lượng online chỉ được cập nhật khi kích thích đo bằng *đạo hàm vận tốc radar* (không dùng IMU thô) đủ để xác định `t_d` với sai số < 20 ms; ngoài lúc đó giữ nguyên `t̂_d` và σ_td (cập nhật kiểu Schmidt). Chi tiết ở Phần E của [báo cáo nhóm](../README.md).
2. **Bộ kiểm tra timestamp độc lập với bộ lọc:** khi |gia tốc| lớn, tương quan chéo giữa gia tốc suy ra từ radar và gia tốc IMU cho một ước lượng độ trễ riêng; lệch > 20 ms so với giá trị đang dùng thì ghi log và báo AEB rằng radar kém tin cậy.
3. **Đưa sai số vị trí mục tiêu và kịch bản "chạy đều rồi phanh" vào bộ test hồi quy bắt buộc**, vì đây là nơi metric thông thường che lỗi. Không chấp nhận một bản cập nhật chỉ dựa trên kết quả lái trong phố.

**Tiêu chí chấp nhận ở vòng sau** (chạy lại đúng benchmark ở phần 3):

- E5: vận tốc ego RMSE khi phanh ≤ 0,05 m/s (hiện tại: 0,64 không bù, 1,03 với TC).
- E5: sai số vị trí mục tiêu khi chạy đều 25 m/s < 0,5 m (tức trễ còn lại < 20 ms).
- E5: \|sai số `t_d`\| / σ_td lúc bắt đầu phanh ≤ 3 (hiện tại ~48).
- Bộ kiểm tra timestamp phát hiện trường hợp không bù trong 1 s đầu pha phanh (kiểm trên log từng scan).
- E1 không xấu đi: offset error ≤ 3 ms, hội tụ ≤ 8 s.

**Dữ liệu cần thu thêm.** Timestamp phần cứng của radar (thời điểm phát chirp) song song với timestamp driver để đo trễ trực tiếp; một chuỗi xe thật có đoạn chạy đều dài rồi phanh mạnh, với GNSS-RTK làm ground truth vận tốc và nhãn vị trí mục tiêu; chạy code EKF-RIO-TC gốc trên chuỗi đó, ghi `t̂_d`, σ_td, NIS và sai số vận tốc để xác nhận hiện tượng trôi mà nhóm thấy trong mô phỏng.

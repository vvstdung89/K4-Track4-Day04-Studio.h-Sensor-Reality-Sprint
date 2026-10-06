# Báo cáo cá nhân T4: Đọc bài báo EKF-RIO-TC, đối chiếu với benchmark của nhóm và đề xuất hướng cải tiến

**Người viết:** Lương Sỹ Khánh (MSSV 2A202602715)  
**Bài thực hành:** Track 4, Ngày 4 (Sensor Reality Sprint), chủ đề T4: Time sync / motion compensation  
**Vai trò trong nhóm:** phụ trách research: chọn và đọc bài báo, tóm tắt phương pháp và benchmark của bài báo, tìm tài liệu liên quan làm cơ sở cho đề xuất cải tiến (Phần E). Phân công của cả nhóm ở [`TEAMMATES.md`](../TEAMMATES.md).  
**Bằng chứng chung của nhóm:** [báo cáo nhóm](../README.md), thư mục [`results/`](../results/) (bảng số, log, hình), code ở commit [`1afade6`](https://github.com/vvstdung89/K4-Track4-Day04-Studio.h-Sensor-Reality-Sprint/commit/1afade6).

Quy ước: 
- "Bài báo cho biết": kết luận của tác giả bài báo
- "Nhóm đo được": kết quả nhóm tự chạy trong mô phỏng
- "Giả thuyết": chưa đo trực tiếp, cần kiểm chứng

## 1. Problem

| | |
| --- | --- |
| **Nền tảng** | Xe có ADAS, ước lượng chuyển động bằng radar 4D + IMU (radar-inertial odometry) |
| **Tính năng bị ảnh hưởng** | Vận tốc ego và vị trí mục tiêu radar, là đầu vào của AEB, ACC và radar tracking |
| **Sensor** | Radar TI AWR1843BOOST và IMU, không có đồng bộ phần cứng |
| **Lỗi thực tế** | Radar xử lý xong cả khung (FFT, beamforming, phát hiện mục tiêu) rồi mới gửi đi; driver gán timestamp lúc nhận nên toàn bộ thời gian này thành độ trễ |

Bài báo cho biết độ trễ radar-IMU khoảng 113 ms, so với 47 ms của camera và 6 ms của LiDAR. Cả dữ liệu tự thu lẫn ColoRadar đều không có đồng bộ phần cứng radar-IMU.

**Câu hỏi research.** Bài báo khẳng định điều gì và trong điều kiện nào; có điều kiện nào một hệ ADAS sẽ gặp mà bài báo không nói tới?

**Claim cần kiểm tra.** Khi radar trễ 50-200 ms, sai số vị trí mục tiêu ≈ *tốc độ × độ trễ* và sai số vận tốc ego ≈ *gia tốc × độ trễ*. Bù trễ online theo bài báo đưa các sai số này về gần mức không trễ, với điều kiện xe có đủ chuyển động.

## 2. Method

**Nguồn truy vết**

| | |
| --- | --- |
| Bài báo | Kim, Bae, Shin, Wang, Oh, *EKF-Based Radar-Inertial Odometry with Online Temporal Calibration*, IEEE RA-L 10(7), 2025. [arXiv 2502.00661v2](https://arxiv.org/abs/2502.00661v2) (v2, 10/06/2025) |
| Repo của tác giả | [spearwin/EKF-RIO-TC](https://github.com/spearwin/EKF-RIO-TC) (C++, ROS Noetic). **Nhóm không chạy**: cần Ubuntu 20.04, catkin, rosbag; máy benchmark chạy Windows. Vì vậy không ghi commit của repo gốc |
| Bản cài lại của nhóm | [`rio_tc_sim.py`](../rio_tc_sim.py), EKF-RIO-TC 2-D bằng Python, commit [`1afade6`](https://github.com/vvstdung89/K4-Track4-Day04-Studio.h-Sensor-Reality-Sprint/commit/1afade6) |

**Lý do chọn bài.** Có code và dữ liệu công khai; dùng EKF nên nối sang Day 5; có metric rõ (offset error bằng giao thức chèn trễ). Bài mới hơn [LC-RIO-ET](https://arxiv.org/html/2603.19958) (03/2026, factor graph + B-spline) tốt hơn khoảng 18,3 % về RPE nhưng phức tạp hơn nhiều, không hợp phạm vi 120 phút.

**Thuật toán.** Độ trễ `t_d` được đưa vào trạng thái EKF, ước lượng cùng hướng, vận tốc và bias IMU. Mỗi scan radar, bộ lọc tích phân IMU tới `t + t̂_d`, dự đoán vận tốc radar lẽ ra phải đo, rồi so với phép đo thật. Cột Jacobian của `t_d` là `H_td = H_q (ω_m − b̂_g) + H_v [R̂ (a_m − b̂_a) + g]`, tức tốc độ thay đổi của đại lượng đo: xe càng tăng tốc hoặc quay thì `t_d` càng quan sát được. Ý tưởng này kế thừa Li & Mourikis (IJRR 2014) cho camera-IMU.

| | |
| --- | --- |
| **Input** | IMU (gia tốc, vận tốc góc) và vận tốc ego radar mỗi scan (RANSAC trên Doppler) |
| **Output** | `t̂_d` kèm σ_td, vận tốc ego, quỹ đạo |
| **Giả định** | `t_d` gần như hằng số; điểm động và điểm ma đã được RANSAC loại; xe có đủ kích thích |
| **Limitation (bài báo cho biết)** | `t_d` không xác định được khi đứng yên; nhiễu quá trình lớn thì hội tụ nhanh nhưng dao động, nhỏ thì ổn định nhưng chậm |

**Khoảng trống thấy được trong khi đọc tài liệu liên quan.** Các nghiên cứu về observability (Yang et al., T-RO 2023; Schneider et al. 2019; OA-LICalib, T-RO 2022) đều coi việc một tham số hiệu chỉnh có quan sát được hay không là điều kiện của **chuyển động**, cần được kiểm tra tường minh và chặn cập nhật khi không đủ kích thích. Bài báo EKF-RIO-TC chỉ nêu điều này định tính và không có cơ chế chặn. Từ đó có giả thuyết: *khi xe chạy đều lâu, bộ lọc vẫn cập nhật `t_d` từ nhiễu và ước lượng sẽ sai mà vẫn tự tin.* Phần 4 kiểm tra giả thuyết này.

## 3. Benchmark

**Dữ liệu và cấu hình.** Dữ liệu **tổng hợp hoàn toàn**, sinh trong `rio_tc_sim.py`, không dùng dataset ngoài. Xe 2-D không trượt ngang; IMU 200 Hz (nhiễu 0,10 m/s² và 0,005 rad/s, có bias); radar 15 Hz (nhiễu 0,05 m/s, gắn cách IMU 3,5 m). Hai kiểu lái: phố 60 s (±2,4 m/s², tốc độ quay tới 0,25 rad/s) và cao tốc 25 m/s chạy đều 40 s rồi phanh −6 m/s². Mỗi số là trung bình 10 seed (0-9), riêng ablation E6 là 20 seed.

**Baseline và điều kiện lỗi.** Baseline `t_d` = 0. Điều kiện lỗi chỉ đổi độ trễ radar (50, 113, 150, 200 ms). Mỗi điều kiện so ba cách xử lý: không bù (timestamp lúc nhận), phương pháp bài báo (TC), oracle (biết `t_d` thật).

**Metric (càng nhỏ càng tốt).**

| Metric | Cách tính | Đơn vị |
| --- | --- | --- |
| Sai số vị trí mục tiêu | Mục tiêu tĩnh cách 30 m, đặt bằng pose *thật* tại thời điểm hệ thống *nghĩ* là lúc đo (chỉ đo lỗi do thời gian) | m |
| Sai số vận tốc ego | RMSE giữa vận tốc ước lượng và thật | m/s |
| Tỉ lệ scan bị loại (proxy ghost rate) | Tỉ lệ scan có NIS > 9,21 (χ²₂ 99 %) | % |
| RPE 10 m | Sai số quỹ đạo tương đối trên đoạn 10 m | m |
| Offset error | \|`t̂_d` − `t_d`\| | ms |

**Bảng chính: lái trong phố, trễ thật 113 ms.** Nguồn: [`results/results.md`](../results/results.md) (E1) và [`results/position_error.md`](../results/position_error.md); dữ liệu tổng hợp, trung bình 10 seed; sai số vị trí lấy sau 10 s đầu, các metric khác lấy trên cả chuyến.

| Cách xử lý | Sai số vị trí [m] | Vận tốc ego RMSE [m/s] | Scan bị loại [%] | RPE 10 m [m] |
| --- | --- | --- | --- | --- |
| Baseline, không trễ | 0 (theo định nghĩa) | 0,018 | 1,2 | 0,044 |
| Không bù (mức lỗi) | 1,31 | 0,154 | 46,0 | 0,196 |
| **Bài báo (TC)** | **0,03** | **0,025** | **1,3** | **0,047** |
| Oracle | 0 | 0,018 | 1,2 | 0,044 |

Offset error của TC là 2,8 ms, hội tụ sau 5,7 s, và hội tụ về cùng giá trị từ khởi tạo 0, −250 hay +100 ms (E2).

**Đối chiếu với kết quả của bài báo.**

| Kết quả | Bài báo cho biết (dữ liệu thật) | Nhóm đo được (mô phỏng) | Đọc thế nào |
| --- | --- | --- | --- |
| `t̂_d` sau hội tụ | −113 ± 2 ms, nhiều giá trị khởi tạo | cùng giá trị từ các khởi tạo khác nhau | Cùng xu hướng |
| Offset error | khoảng 15 ms | 2,8 ms | Không so trực tiếp: mô phỏng lý tưởng hơn |
| APE / RPE tịnh tiến so với EKF-RIO | giảm 56 % / 50 % | giảm 57 % / 76 % | Chỉ so xu hướng |
| Trễ thật bằng 0 | ngang baseline | RPE 0,046 so với 0,044 m | Thêm `t_d` vào trạng thái là an toàn |

**Kiểm tra bản cài lại.** Jacobian khớp sai phân hữu hạn tới 9,9e-10 ([`check_claims.txt`](../results/check_claims.txt)); trên đường thẳng, sai số vị trí khớp đúng công thức tốc độ × độ trễ ở mọi ô 5-30 m/s × 50-200 ms.

**Lệnh chạy.** `python rio_tc_sim.py` (khoảng 8 phút), rồi `python position_error.py`, `python export_failure_log.py`, `python plot_delay_impact.py`, `python check_claims.py`. Môi trường: Python 3.10.11, numpy 2.2.6, matplotlib 3.10.9.

**Giới hạn của benchmark (threats to validity).** Mô phỏng 2-D, không có RANSAC, nhiễu Gauss lý tưởng, không rung hay sai ngoại tham số; nhóm cài lại chứ không chạy code gốc, nên khác biệt với bài báo có thể do bản cài lại. Tỉ lệ scan bị loại chỉ là proxy, không phải số mục tiêu ma thật; sai số vị trí dùng pose thật nên chưa gồm sai số odometry; benchmark không đo AEB hay tracker thật.

## 4. Failure case: chạy đều 40 s rồi phanh gấp, phương pháp bài báo còn tệ hơn không bù

**Tình huống.** Xe chạy thẳng đều 25 m/s trong 40 s, rồi phanh −6 m/s² xuống 5 m/s. Độ trễ thật 113 ms, không đổi. Đây là kịch bản kiểm tra giả thuyết ở phần 2: lúc chạy đều không có kích thích, nên `t_d` không quan sát được.

![Ảnh hưởng của trễ radar tới vận tốc ego](../results/fig5_delay_impact.png)

*Hình 5 ([`results/fig5_delay_impact.png`](../results/fig5_delay_impact.png)). Phải: kịch bản này, một lần chạy (seed 1). Log từng scan: [`results/e5_failure_log_seed1.csv`](../results/e5_failure_log_seed1.csv).*

**Nhóm đo được** (E5, trung bình 10 seed, [`results/results.md`](../results/results.md)):

| Cách xử lý | Sai số `t_d` lúc bắt đầu phanh [ms] | σ_td bộ lọc báo [ms] | Vận tốc ego RMSE khi chạy đều [m/s] | Vận tốc ego RMSE khi phanh [m/s] |
| --- | --- | --- | --- | --- |
| Không bù | 113 | – | 0,018 | 0,639 |
| TC, khởi tạo 0 | 320 | 6,7 | 0,018 | **1,028** |
| TC, khởi tạo đúng −113 ms | 224 | 7,0 | 0,018 | 0,731 |
| Oracle | 0 | – | 0,018 | 0,018 |

- Khi phanh, TC sai vận tốc 1,03 m/s, **tệ hơn không bù 61 %**. Khởi tạo đúng vẫn không cứu được.
- Bộ lọc **sai mà tự tin**: sai số `t_d` gấp khoảng 48 lần σ_td báo cáo.
- Vị trí mục tiêu ở giây cuối của đoạn chạy đều sai 8,4 m với TC (không bù: 2,8 m; [`position_error.md`](../results/position_error.md), P3).
- Không có dấu hiệu báo trước: khi chạy đều, mọi phương pháp có cùng sai số vận tốc 0,018 m/s. Lúc phanh chỉ 13 % scan bị gating loại, vì bộ lọc khớp vận tốc với phép đo đã lệch thời gian.

**Kiểm tra giả thuyết.** Thí nghiệm E3 (biên độ gia tốc 0) cho thấy cùng hiện tượng không cần đến kịch bản phanh: `t̂_d` trôi 318 ms trong khi σ_td báo 10,6 ms, và `t_d` chỉ hội tụ 10/10 lần khi gia tốc từ 1 m/s² ([hình 2](../results/fig2_e3_excitation.png)). Ablation E6 (20 seed) xác định nguyên nhân: tắt nhiễu gyro giảm độ trôi từ +313 ms xuống +107 ms, bỏ cánh tay đòn radar xuống +89 ms, tắt mọi nhiễu IMU (giữ bias) xuống +73 ms. Cả hai cơ chế đều xuất phát từ việc `H_td` tính bằng phép đo IMU **thô**, đúng công thức bài báo:

- *Quan sát được giả:* lúc không tăng tốc hay quay, `H_td` lẽ ra bằng 0 nhưng nhiễu và bias làm nó khác 0, nên EKF coi là kích thích thật và thu nhỏ σ_td dù dữ liệu không chứa thông tin.
- *Nhiễu tương quan:* cùng một mẫu gyro đi vào cả dự đoán phép đo (qua `ω × p_R`) lẫn `H_td`, nên bước cập nhật có kỳ vọng khác 0 và đẩy `t_d` về một phía.

**So với bài báo.** Bài báo chỉ nói `t_d` "không xác định được khi đứng yên". Nhóm đo được tình huống xấu hơn: `t̂_d` không đứng yên mà trôi, và hiệp phương sai báo sai độ tin cậy.

**Giả thuyết chưa kiểm chứng.** Dữ liệu thật của bài báo là cầm tay và drone, vốn luôn có chuyển động quay, nên hiện tượng này có thể không xuất hiện ở đó. Nhóm chưa có dữ liệu để xác nhận hay bác bỏ.

**Ảnh hưởng tới tính năng (suy luận, chưa đo).** Khi phanh, AEB và ACC nhận vận tốc ego cao hơn thật khoảng 0,7 m/s (khớp ước tính 6 × 0,113 ≈ 0,68 m/s khi không bù). Ở 25 m/s, mục tiêu bị đặt sai 2,8 m khi không bù, hơn nửa làn đường (1,75 m); để sai dưới 0,5 m (ngưỡng nhóm tự chọn) thì độ trễ còn lại phải dưới 20 ms.

## 5. Engineering decision

**Quyết định.** Phải bù trễ radar, nhưng không dùng ước lượng online của bài báo như một hộp đen. Cho `t_d` học chỉ khi có kích thích đủ lớn, đo bằng tín hiệu không dùng chung nhiễu với bước cập nhật; ngoài lúc đó giữ nguyên `t̂_d` và σ_td.

| Tình huống | Quyết định | Căn cứ (nhóm đo được) |
| --- | --- | --- |
| Phố, gia tốc thường xuyên từ 1 m/s² | Dùng ước lượng online | Hội tụ sau 3-13 s; vị trí 1,31 → 0,03 m; scan bị loại 46 % → 1,3 % |
| Cao tốc chạy đều lâu | Không để bộ lọc tự học `t_d` | Sau 40 s chạy đều: vị trí sai 8,4 m, vận tốc lúc phanh sai 1,03 m/s |
| Trễ đã đo offline, ổn định | Dùng giá trị cố định, chỉ giám sát | Bù đúng `t_d` cho kết quả bằng oracle |

**Cải tiến đề xuất: cập nhật `t_d` có nhận biết khả quan sát (chưa cài đặt).** Lấy ý tưởng từ nhóm tài liệu observability-aware ở phần 2, nhắm đúng cơ chế đã đo ở phần 4:

1. Đo kích thích bằng **đạo hàm vận tốc ego của radar** trong cửa sổ 1 s (median), không dùng IMU thô, nên không lặp lại hai cơ chế ở phần 4.
2. Chỉ mở cổng cập nhật `t_d` khi cửa sổ đủ thông tin để xác định `t_d` với sai số dưới 20 ms (tương ứng gia tốc RMS khoảng 0,65 m/s², khớp điểm E3 bắt đầu hội tụ và khớp ngưỡng 20 ms ở cao tốc).
3. Cổng mở: tính `H_td` bằng IMU trung bình trên khoảng 50 ms để cắt đường tương quan nhiễu. Cổng đóng: cập nhật kiểu Schmidt, giữ nguyên `t̂_d` và σ_td.

Rủi ro: chọn ngưỡng cổng (cao thì phí kích thích nhẹ, thấp thì học từ nhiễu); cổng đóng lâu thì không theo kịp khi trễ đổi giữa chuyến (nhóm đo được ở E4: trễ tăng 50 ms thì bộ lọc không bắt kịp trong 45 s); đạo hàm radar phụ thuộc chất lượng RANSAC.

**Fallback.** Nếu cổng đóng quá 10 s, đánh dấu ước lượng online "không tin cậy" và dùng `t_d` hiệu chỉnh offline cho từng loại radar. Song song, một bước kiểm tra timestamp độc lập với bộ lọc: tương quan chéo giữa gia tốc suy ra từ radar và gia tốc IMU; lệch quá 20 ms so với giá trị đang dùng thì ghi log và báo AEB rằng radar kém tin cậy.

**Tiêu chí chấp nhận** (chạy lại đúng benchmark ở phần 3):

| Kiểm tra | Hiện tại | Mục tiêu |
| --- | --- | --- |
| E5: vận tốc ego RMSE khi phanh [m/s] | 0,639 không bù; 1,028 TC khởi tạo 0 | ≤ 0,05 |
| E5: \|sai số `t_d`\| / σ_td lúc bắt đầu phanh | khoảng 48 | ≤ 3 |
| E3, gia tốc 0: sai số `t_d` sau 60 s [ms] | 318 | không trôi khỏi giá trị khởi tạo |
| E1: offset error [ms] / hội tụ [s] | 2,8 / 5,7 | ≤ 3 / ≤ 8 |

**Log cần ghi.** Mỗi scan: `t̂_d`, σ_td, NIS, vận tốc ego, và tín hiệu kích thích đo bằng radar (xem log mẫu [`e5_failure_log_seed1.csv`](../results/e5_failure_log_seed1.csv)).

**Dữ liệu cần thu thêm.**

- Một chuỗi xe thật có đoạn dài chạy đều rồi phanh mạnh, với GNSS-RTK làm ground truth vận tốc và nhãn vị trí mục tiêu.
- Chạy code EKF-RIO-TC gốc trên chuỗi đó, ghi `t̂_d`, σ_td, NIS, để xác nhận hoặc bác bỏ hiện tượng trôi; đối chiếu σ_td khởi tạo và nhiễu quá trình với bản cài lại, vì độ lớn của hiện tượng có thể phụ thuộc các tham số này.
- Timestamp phần cứng của radar (thời điểm phát chirp) song song với timestamp driver, để đo trễ trực tiếp.

## Tài liệu tham khảo

- Kim, Bae, Shin, Wang, Oh, ["EKF-Based Radar-Inertial Odometry with Online Temporal Calibration"](https://arxiv.org/abs/2502.00661), IEEE RA-L 10(7), 2025. Code: [EKF-RIO-TC](https://github.com/spearwin/EKF-RIO-TC).
- Štironja, Petrović, Peršić, Marković, Petrović, ["Radar-Inertial Odometry with Online Spatio-Temporal Calibration via Continuous-Time IMU Modeling" (LC-RIO-ET)](https://arxiv.org/html/2603.19958), arXiv, 03/2026.
- Doer & Trommer, EKF-RIO và dữ liệu ICINS2021: [repo rio](https://github.com/christopherdoer/rio).
- Li & Mourikis, ["Online temporal calibration for camera-IMU systems: Theory and algorithms"](https://intra.ece.ucr.edu/~mourikis/papers/Li2014IJRR_timing.pdf), IJRR 33(7), 2014.
- Yang, Geneva, Zuo, Huang, ["Online Self-Calibration for Visual-Inertial Navigation: Models, Analysis, and Degeneracy"](https://arxiv.org/abs/2201.09170), IEEE T-RO 39(5), 2023.
- Schneider, Li, Cadena, Nieto, Siegwart, ["Observability-Aware Self-Calibration of Visual and Inertial Sensors for Ego-Motion Estimation"](https://arxiv.org/abs/1901.07242), 2019.
- Lv, Zuo, Hu, Xu, Huang, Liu, ["Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems" (OA-LICalib)](https://arxiv.org/abs/2205.03276), IEEE T-RO 2022.

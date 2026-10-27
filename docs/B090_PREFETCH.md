# B090 prefetch — 26/10/2026

Điều chỉnh theo yêu cầu tiếp theo của người vận hành: target đã READY sau B090
được chốt tọa độ và dùng ngay sau Z+200, không lấy frame hoặc xác nhận YOLO lại.

## Phạm vi và kết quả audit

Thực hiện trong workspace `C:\Data\Picking_full_cycle` được cung cấp. Bản tham
chiếu `nx100_variables_web` từng được cung cấp trong workspace; lúc audit chỉ đọc
`nx100_client.py`, `app.py` và test tham chiếu, không import vào ứng dụng.
Thư mục tham chiếu đã được xóa theo yêu cầu dọn project sau triển khai B090.
Không kết nối NX100,
camera thật, không chạy server REAL và không sửa dữ liệu vận hành.

Transport hiện có là `robot.protocol.Client.execute()`, một kết nối TCP mỗi lệnh,
port theo cấu hình hiện có, khóa `_WIRE_LOCK`. Controller có `RLock` tuần tự hóa
command và telemetry. SAVEV trước đây chỉ cho phép P009/P010. Prefetch trước đây
được bật trong `_prepare_drop_variable()` trước START DROP, tích lũy `StableTarget`
trên cùng vision worker; `_target()` lấy lịch sử và xác minh bằng frame mới sau nâng.

Thứ tự robot được giữ nguyên: `_job()` xác nhận MOVEUNTI → `_execute_hand_job()`
chờ H2ON → `_prepare_drop_variable()` đọc P009, tính Z, ghi và readback P010 →
START DROP → `_wait_drop_variable_target()` xác nhận dừng tại P010 → H2OFF →
`StackStore.confirm()` → `_after_h2off_move()` nâng Z+200 và chờ reached/stopped →
`_run_batch()` tạo cycle ID mới → `_target()` → `_move(..., 'MOVE_XY')`.

## File thay đổi

| File | Nội dung |
|---|---|
| `robot/protocol.py` | Chỉ thêm SAVEV `0,90` vào validator; LOADV vẫn chỉ cho P010 |
| `robot/controller.py` | Thêm `read_b090()`, connection generation khi connect |
| `robot/mock.py` | Mô phỏng B090 reset và checkpoint theo thời gian DROP |
| `vision/pipeline.py` | Context cycle, identity, READY, consume và freshness cho prefetch hiện có |
| `workflow/cycle.py` | Arm/poll B090; kiểm tra provenance; log và snapshot chẩn đoán |
| `workflow/drop.py` | Bỏ trigger cũ, poll trong vòng chờ DROP; log các mốc hoàn thành |
| `tests/test_b090_prefetch.py` | 22 test mock bổ sung |
| `tests/test_repeat.py` | Mô phỏng cùng cảnh ở background prefetch và sample, camera loss thực sự |
| `docs/B090_PREFETCH.md` | Báo cáo này |

Các hàm mới: `Controller.read_b090()`, `Cycle._arm_drop_prefetch()`,
`Cycle._poll_drop_prefetch()`, `Cycle._prefetch_event()`,
`Vision.preparation_identity()`, `Vision._update_next_pick()`,
`Vision.prefetch_status()`, `Vision.consume_next_pick()`, `Vision.take_ready_pick()`.
Các hàm prepare/take/clear hiện có được mở rộng; không có worker mới.

## SAVEV và xác thực lượt DROP

`read_b090()` thực hiện `client.execute('SAVEV', '0,90')` trong controller lock,
parse số nguyên ASCII 0–255, không cache và không ghi B090. Byte ngoài 0/1 hợp lệ
về giao thức nhưng không được nhận làm checkpoint. Handshake, framing, timeout và
OutcomeUnknown của lệnh điều khiển giữ nguyên. Lỗi đọc chỉ vô hiệu hóa prefetch;
kiểm tra robot của workflow chính vẫn quyết định HOLD/FAULT theo cơ chế cũ.

Sau START DROP đã được ACK, tạo context RAM mới theo cycle ID và connection
generation, xóa lịch sử prefetch cũ. Chỉ đọc khi RJSEQ xác nhận JOB hiện tại DROP.
Phải quan sát B090=0 sau START được chấp nhận, sau đó đọc được 1 trên cùng context
mới bật prepare. START ACK hoặc một giá trị 1 riêng lẻ không chứng minh reset.
Đổi cycle, reconnect, lỗi parse/timeout hoặc mất kết nối vô hiệu hóa context.

Không dùng cache từ telemetry và không đọc P009 để kích hoạt vision. P009 vẫn
chỉ được đọc phục vụ phép tính/ghi P010 theo workflow cũ.

**Giới hạn:** không có bằng chứng tương đương khác cho reset trong source hiện có.
Nếu TCP polling bỏ lỡ khoảng 0, backend không suy đoán B090=1 thuộc lượt mới.
Khi DROP hoàn thành mà chưa xác thực đủ, bỏ prefetch và nhận diện/ổn định từ đầu
sau Z+200. Không tự ghi 0, không sửa JOB để giải quyết giới hạn này.

## Xử lý song song và dispatch

Ngay khi trigger hợp lệ, `prepare_next_pick()` thay bộ lịch sử cũ bằng StableTarget
mới, gắn source cycle ID. Chỉ dùng capture timestamp sau trigger. Cùng camera,
detector, tracker và vision worker tiếp tục tính XY và tích lũy ổn định trong
DROP/H2OFF/nâng; READY chỉ được ghi khi `StableTarget.update()` trả target hợp lệ.
Tại READY, lưu cả XY và pose MOVL. Tọa độ được chốt, các frame tiếp theo không
thay thế hoặc xác nhận lại target này. Worker này không có đường phát MOVE.

Sau nâng đã được `_move()` xác minh reached/stopped, `_target()` đối chiếu source
cycle với `last_completed_drop_cycle_id`, thứ tự ID, connection generation,
config fingerprint và calibration object/ma trận/nội dung file. Nếu đã READY,
`take_ready_pick()` lấy pose đã chốt đúng một lần, chuyển PREPARED và `_target()`
trả pose ngay. Không gọi `vision.sample()`, không cần frame sau nâng, không kiểm
tra lại cảnh bằng YOLO và không loại tọa độ READY chỉ vì thời gian DROP kéo dài.
Giả định vận hành do người dùng xác nhận: sau checkpoint B090, tay robot đã ở khu
DROP và khu PICK trống, ổn định. ID/instance, tolerance, freshness/gap và thời gian
ổn định vẫn được xác minh trong lúc chuẩn bị trước READY. Nếu chưa READY, tiếp tục
nhánh tích lũy lịch sử/frame mới hiện có. XY trùng lượt trước được phép.

Trước MOVL tiếp tục kiểm tra cờ mất kết nối camera/identity/kết nối và các guard robot, pose,
workspace, cancel, journal hiện có. Chỉ cycle worker phát lệnh; START trùng và
process thứ hai vẫn bị khóa. Sau MOVL được chấp nhận, đánh dấu CONSUMED; mất phản
hồi vẫn đi theo OutcomeUnknown cũ, không retry. Camera lỗi, HOLD, FAULT, đóng
worker hoặc restart đều không giữ target dùng lại.

Snapshot API bổ sung `prefetch` gồm B090, source cycle, reset evidence, state,
reason và `vision` (target, frame, age_seconds, ready_at). Giao diện và các nút
điều khiển giữ nguyên. `target_wait_seconds` hiện có ghi thời gian chờ trong
`_target()` sau bước kiểm tra đầu lượt mới, không phải tổng thời gian truyền lệnh.
Timestamp prefetch dùng monotonic trong process, không khôi phục qua restart.

Log theo cycle ID: DROP_STARTED (START đã ACK), B090_RESET_CONFIRMED (đã đọc 0),
B090_TRIGGERED, PREFETCH_STARTED, NEXT_TARGET_READY, DROP_COMPLETED,
H2OFF_COMPLETED, Z200_COMPLETED, NEXT_TARGET_LOADED, NEXT_MOVL_SENT.
NEXT_TARGET_VALIDATED chỉ còn ở nhánh lịch sử chưa READY cần tiếp tục ổn định.
B090 không phải tín hiệu hoàn thành nâng và không cấp quyền MOVE.

## Regression và kiểm thử

Giữ nguyên model, camera settings, calibration assets/transform/offset,
StableTarget thresholds, tracker/ID priority, MOVEUNTI/laser, H2ON/H2OFF, JOB DROP,
P009 ownership, quy trình P010, tốc độ, MOVL và nâng Z+200. Z thả vẫn
`330 + stacked_count * board_thickness_mm` với mặc định 12 mm; plane_z_mm tách biệt.
Không thay stack count, journal, operator settings hoặc log lịch sử REAL.

| Nhóm yêu cầu test | Test kiểm chứng |
|---|---|
| 1–4: SAVEV 0/1, timeout, parse | `ByteReadTests` (bao gồm 255, từ chối ghi byte) |
| 5–7: 1 cũ, fallback, trigger hợp lệ | `CheckpointTests`: old_high, missing_reset, valid_reset, read_failure |
| 8–11: empty/unstable/READY/same XY | CheckpointTests, OverlapTests, `test_prefetch.py`, `test_cycle.py` |
| 12–13: chỉ MOVE sau nâng, một lần | OverlapTests: thứ tự log, đúng bốn MOVL cho hai lượt |
| 14–17: stale/camera/calibration/config | CheckpointTests và `test_prefetch.py` |
| 18–21: H2OFF lỗi/HOLD/ALARM/NX disconnect | `test_drop.py`, CheckpointTests, `test_cycle.py`, `test_repeat.py` |
| 22–23: restart/worker độc quyền | CheckpointTests, duplicate START và second server trong `test_cycle.py` |
| 24–26: Z_drop/P009/P010 readback | `test_position_variable.py`, `test_drop.py` |
| 27–30: MOVEUNTI/HAND/Z+200/count một lần | `test_laser_h2_cycle.py`, `test_hand_jobs.py`, `test_drop.py`, `test_repeat.py` |

Tất cả thiết bị trong test dùng MockClient hoặc FakeSocket. Lệnh chạy hồi quy có
thêm patch `socket.create_connection` mặc định ném AssertionError; các test framing
chỉ thay nó bằng FakeSocket riêng. Test relocation chạy subprocess ở thư mục tạm,
được hard-code `mock=True`. Không có kết nối tới robot thật.

Kết quả trước điều chỉnh: 172/172 test đạt. Sau điều chỉnh, nhóm B090/prefetch/repeat
chạy riêng đạt 32/32. Test tích hợp làm chậm DROP mô phỏng để kiểm chứng READY
trước DROP_COMPLETED và xác minh Z200_COMPLETED → NEXT_TARGET_LOADED →
NEXT_MOVL_SENT, đúng một MOVE XY/lượt. Test sẽ thất bại nếu sample() được gọi cho
lượt thứ hai đã READY; test khác xác minh tọa độ được giữ qua frame trống và quá
max_frame_age.

Hồi quy toàn bộ trên bản hiện tại: **173/173 test đạt**, 0 failure, 0 error,
0 skipped. Kết quả hồi quy toàn bộ sau khi gom thư mục được lưu trong
[`LAYOUT_REGRESSION_RESULT.json`](LAYOUT_REGRESSION_RESULT.json), thời gian 91,672 giây.
Bản sửa hiện tại đã được rà lại theo luồng B090 → READY chốt
tọa độ → Z200_COMPLETED → NEXT_TARGET_LOADED → NEXT_MOVL_SENT. Không viết lại
kiến trúc hoặc thay quy trình robot khi hoàn tất lần rà soát này.

Lệnh chạy từ thư mục project (chặn TCP thật mặc định):

```powershell
.venv/Scripts/python.exe -c "import logging,unittest; from unittest.mock import patch; logging.getLogger().addHandler(logging.NullHandler()); suite=unittest.defaultTestLoader.discover('tests'); guard=patch('socket.create_connection',side_effect=AssertionError('Network disabled during mock tests')); guard.start(); result=unittest.TextTestRunner(verbosity=1).run(suite); raise SystemExit(not result.wasSuccessful())"
```

## Trước khi chạy REAL

Người vận hành cần đối chiếu trên máy rằng SAVEV `0,90` đọc đúng byte, RJSEQ đúng
JOB và khoảng reset 0 đủ quan sát với độ trễ TCP hiện tại. Theo dõi thứ tự log và
đo thời gian chờ target sau nâng. Nếu không bắt được reset, fallback là hành vi dự
kiến; không có cam kết giảm thời gian ở mọi lượt.

Chưa kiểm chứng tốc độ YOLO thực tế, occlusion sau P009, thời gian truyền lệnh,
hiệu quả giảm chu kỳ hoặc thao tác vật lý trên robot. Calibration `needs_review`
đã có trước bản sửa và không được thay đổi. Phiên làm việc này không sửa/upload
JOB, không thay ladder, không ghi B090 và không tự triển khai hoặc vận hành REAL.

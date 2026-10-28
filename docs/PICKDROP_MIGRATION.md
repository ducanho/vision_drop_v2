# SINGLE JOB PICKDROP — 27/10/2026

Phạm vi: chỉnh trực tiếp project hiện tại. Kết quả là triển khai và kiểm thử
offline; chưa nghiệm thu robot thật. Không chạy entry point REAL, upload JOB,
gửi lệnh tới NX100, sửa tốc độ/I000, calibration hay dữ liệu vận hành.

## Luồng hiện tại

`START HMI → SYSTEM_CHECK → _target → _prepare_pick_variable →
_prepare_drop_variable → _start_pickdrop → _wait_pickdrop_complete →
_confirm_pickdrop → STACK_COUNT_UPDATED → COMPLETE → repeat/empty`.

- API `/api/cycle/start` và nút START cũ giữ nguyên. `full_cycle` luôn chọn
  PICKDROP; các cờ legacy `end_after_moveunti_start` và `z_completion` không
  đổi nó về MOVEUNTI. Các stage commissioning riêng vẫn có API cũ.
- `_target`, camera, YOLO, tracker, StableTarget, ID/cursor, conversion,
  offsets, settings pose và chính sách READY không thay đổi.
- P002 dùng nguyên hàm cũ: XY target, Z_SAFE/RX/RY/RZ settings và metadata
  RPOSC hiện tại; LOADV rồi SAVEV kiểm tra từng trường.
- P010 dùng nguyên SAVEV P009, chỉ thay Z bằng Decimal
  `first_drop_z_mm + stacked_count * board_thickness_mm`; giữ mọi trường khác.
  LOADV P010 rồi SAVEV verify; workspace và giới hạn DROP SAFE giữ nguyên.
- Cả hai receipt gắn cycle ID/connection generation. Trước START, đọc lại cả
  P002/P010, kiểm tra metadata live, readiness, laser OFF, camera/identity của
  target prefetch và RJDIR có PICKDROP.
- `job_control.start(client, 'PICKDROP')` phát đúng một START. Controller
  whitelist tên này qua `allowed_start_jobs`; framing TCP, locking và
  OutcomeUnknown/no-retry giữ nguyên. `job_start_attempted` chặn lần thử thứ hai.
- Không có START MOVEUNTI/DROP/H2ON/H2OFF, IOWRITE hay MOVL PC trong full-cycle.
  Đã bỏ fallback DROP dùng PC hạ Z, HAND OFF và nâng Z+200.
- Các trạng thái mới START_PICKDROP/PICKDROP_RUNNING/PICKDROP_COMPLETE dùng
  cùng journal và một cycle worker. HMI vẫn dùng busy/COMPLETE/FAULT/HOLD cũ.

## B090 và B091

Đọc uncached qua SAVEV `0,90` / `0,91`; response là số nguyên 0..255. Parser
từ chối malformed/out-of-range. Protocol vẫn cấm ghi hai byte từ Python.

B090 được arm ngay sau START ACK, trước bất kỳ sleep nào. Chỉ chấp nhận 0 → 1
quan sát sau START, trong cùng cycle, connection generation và RJSEQ PICKDROP.
Đọc lỗi/khác JOB/mất reset làm prefetch INVALID và dùng fallback stability;
không dùng 1 cũ, không reset byte, không dùng B090 để count. Vision worker chỉ
chuẩn bị target; không điều khiển robot. READY giữ tọa độ chốt, source cycle,
ID ordering, config/calibration identity và policy không kiểm tra lại tuổi frame.
Chu kỳ mới chỉ lấy READY sau completion/count của chu kỳ nguồn.

B091 được lấy mẫu ngay sau arm và trong loop theo dõi, với identity tương tự.
Phải thấy 0 sau START trước khi chấp nhận 1. 0 là pending; 2 là FAULT chung,
không suy ra chính xác lỗi laser hay vacuum; giá trị khác là UNKNOWN. 1 không
đủ để thành công nếu còn RUNNING, sai JOB, HOLD/alarm/error, sai pose hoặc
pending/outcome_unknown. Reset 1 → 0 trong cùng cycle cũng gây FAULT.

Thành công cần START accepted, B091 reset và 1, JOB PICKDROP, RUNNING OFF,
readiness/telemetry đầy đủ và final pose reached đủ settle_samples. Trước
durable count, đọc lại trạng thái/JOB/B091/pose một lần nữa. StackStore.confirm
giữ chống trùng cycle ID, last_picked_id, last_completed_drop_cycle_id và
Decimal arithmetic. Receipt ghi `board_release_confirmed=False`: đây là kết
quả chương trình, không phải bằng chứng vật lý ván đã rời đầu hút.

FAULT/HOLD không count/repeat, không tự HAND OFF, reset byte hoặc resume.
Mất reply START giữ pending/outcome_unknown, không thử lại. Nhánh lỗi giữ yêu
cầu HOLD một lần của hệ thống cũ; mất truyền thông không được coi HOLD thành
công. Phục hồi yêu cầu kiểm tra thủ công/ACK; ACK không nhả hardware HOLD.
Restart giữa JOB hoặc giữa completion và count đưa về FAULT/outcome_unknown;
không replay. Single-instance lock và tách dữ liệu MOCK/REAL giữ nguyên.

## Hợp đồng JBI và REAL đã mở theo xác nhận người vận hành

Sau migration, người vận hành xác nhận đã xóa `JUMP *TICK` trên NX100 và P011
là IMOV theo **BASE: X/Y giữ nguyên, Z tăng 200 mm**. Bản local
[PICKDROP.JBI](../jobs/PICKDROP.JBI) được đồng bộ bằng cách chỉ xóa dòng
`JUMP *TICK`. Không sửa P003/P008/P009/P011/I000, tốc độ hoặc I/O mapping.

`config.pickdrop` ghi nhận operator_confirmed, SHA256 bản JOB đã đồng bộ và
p011_frame=BASE. `real_gate(full_cycle)` hiện cho phép REAL theo hợp đồng này;
không còn khóa vô điều kiện. Bản export thay đổi, nhãn JUMP chưa định nghĩa,
thiếu xác nhận hoặc frame khác BASE vẫn bị chặn với lý do cụ thể.

Adapter final pose đọc vector P011 thật bằng SAVEV `4,11`, cộng translation
BASE vào P010 để kiểm tra đích cuối. Không hardcode Z+200, không gửi chuyển động
PC thay IMOV. P011 có rotation khác 0 bị chặn vì adapter không suy đoán phép
biến đổi rotation. Mọi guard B091 reset/success, RJSEQ, RUNNING, readiness,
workspace, HOLD/alarm/error, no-retry, count và restart vẫn áp dụng.

MOCK mô phỏng laser, HAND nội bộ, vacuum sensor 1 OR sensor 2, B090/B091 và
final pose; có test chạy nhánh REAL bằng MockClient để không kết nối thiết bị.
Việc mở REAL dựa trên xác nhận người vận hành, không dựa trên tuyên bố đã chạy
thử robot thật. Calibration assets và thuật toán giữ nguyên.

## Nút HOME

HMI bổ sung một nút HOME trong panel Robot. `/api/robot/home` dùng lease,
revision và request ID chống trùng của API hiện có. Backend yêu cầu IDLE hoặc
COMPLETE, cycle không busy, không pending/outcome_unknown, robot connected,
servo ON, remote/play, không RUNNING/HOLD/alarm/error và RJDIR có HOME.

Chỉ gửi `START HOME` một lần tới JOB có sẵn. Không ghi pose, không phát MOVL,
không đổi tốc độ MOVJ của HOME.JBI và không điều khiển HAND. HOME không cần
camera. Sau ACK, state là JOB_DISPATCHED với completion_verified=false, không
tuyên bố robot đã tới HOME. Khi đọc được robot dừng, người vận hành bấm RESET
để ACK về IDLE trước lệnh tiếp theo. Mất reply giữ FAULT/pending/outcome_unknown,
không retry; restart không phát lại. Các nút START/HOME/HAND/servo bị khóa khi
đang bàn giao. HOME.JBI giữ nguyên.

## Kiểm thử

Chạy từ project bằng `.venv\Scripts\python.exe -m tools.run_tests`.
Runner chặn `socket.create_connection` mặc định; wire tests chỉ dùng FakeSocket.
Subprocess relocation cũng chặn socket; mọi Cycle có thực thi test đều dùng
MockClient. Không chạy `app.py` để thử nghiệm.

Syntax/import: `.venv\Scripts\python.exe -m compileall -q app.py core robot vision workflow tests tools`.
Kết quả toàn bộ suite: [test_result.json](../logs/reports/tests/test_result.json),
chi tiết từng test: [test_output.txt](../logs/reports/tests/test_output.txt).
Lượt mới nhất sau mở REAL và thêm HOME: **221/221 PASS**, 0 failure, 0 error,
0 skipped, 49.89 giây theo runner. Browser HOME smoke PASS, không có lỗi JS,
tablet không tràn ngang. Syntax, asset integrity và git diff whitespace check
PASS; kết quả chỉ áp dụng offline.
Các test sequencing cũ được chuyển sang PICKDROP; test wire/metadata,
manual HAND, servo, HOLD release, API, camera, calibration và tracker vẫn chạy.
Integration có kiểm tra READY trước completion, không resample READY, repeat
1/2/3/4/1, dừng khi hết ván và không START lượt sau trước khi count lượt trước.
Kiểm tra UI HOME: `.venv\Scripts\python.exe -m tools.browser_home_smoke` dùng
server MOCK riêng, chặn kết nối robot; test click HOME, thông báo START HOME,
khóa START, RESET về IDLE, desktop/tablet và JS errors. Báo cáo/screenshots:
[home_browser](../logs/reports/home_browser/result.json).

## File và các tham chiếu legacy

Runtime sửa: robot/controller.py, job_control.py, protocol.py, mock.py,
position_variable.py (chỉ thông báo lỗi), vacuum.py (cấm map HAND tới motion
JOB); workflow/cycle.py, drop.py, state_machine.py, safety_checks.py.
Thêm workflow/pickdrop.py. StackStore và hàm tính XY/P002 giữ nguyên.
Theo yêu cầu HOME/mở REAL còn sửa app.py (thêm route), config/config.json,
templates/index.html, static/app.js, static/style.css và xóa đúng JUMP *TICK
trong jobs/PICKDROP.JBI theo thay đổi người vận hành xác nhận.

Test/tool sửa: tests/test_api.py, test_b090_prefetch.py, test_cycle.py,
test_dispatch_cycle.py, test_drop.py, test_independence.py,
test_laser_h2_cycle.py, test_pick_variable.py, test_position_variable.py,
test_repeat.py; tools/run_tests.py. test_servo.py chỉ tăng timeout của test
delayed-readback lên 0.3 giây để đủ ba poll trên Windows; timeout production
và các test timeout không đổi. Thêm tests/pickdrop_support.py và
tests/test_pickdrop.py. Bổ sung tests/test_home.py và tools/browser_home_smoke.py.

Giữ nguyên các route cũ, core/settings.py, toàn bộ vision/, calibration/,
models/, HOME.JBI và dữ liệu vận hành REAL/MOCK đã có. Các test dùng
journal/stack tạm.

MOVEUNTI còn ở job_control default/protocol/mock và methods commissioning
`moveunti_only`, `_job`, `_wait_laser_and_stopped`, `_wait_vacuum_ready`; không
được full_cycle gọi. `config.job` và các cờ handoff chỉ còn phục vụ stage cũ.
DROP trong allowed jobs/mock là compatibility legacy, không có dispatch trong
full-cycle. Các state DROP/H2OFF và API confirm cũ giữ cho journal/API
compatibility. Manual HAND jobs và wire tests cũ không bị xóa chỉ vì tên cũ.
Các tài liệu kỹ thuật trước migration là lịch sử; tài liệu này mô tả full-cycle
hiện tại. Không có nhánh full-cycle thứ hai hoặc worker điều khiển thứ hai.

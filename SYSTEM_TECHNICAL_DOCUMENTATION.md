# VISION_PICKING — SYSTEM TECHNICAL DOCUMENTATION

## AI QUICK CONTEXT

**Kiến trúc: SINGLE JOB PICKDROP.** Làm việc ngay trong folder vision_picking, không tạo project lồng hoặc sao chép sang dự án khác. Phần cứng theo ngữ cảnh vận hành: Yaskawa Motoman UP50N/NX100, camera HIKROBOT GigE cố định, laser và đầu hút. PC Windows chạy Python/FastAPI, MVS, YOLO và Eye-to-Hand; NX100 sở hữu toàn bộ chuyển động gắp–thả trong một JOB.

**Flow chính:** YOLO → tracker/target ổn định → tính XY → ghi/verify P002 → StackStore/P009 tính Z thả → ghi/verify P010 → **START PICKDROP một lần** → B090 prefetch → B091 + JOB/stopped/final pose → confirm count → repeat tới hết ván. Full cycle không START MOVEUNTI/DROP/H2ON/H2OFF riêng, không gửi MOVL thay P008/P011. Stage commissioning/manual cũ vẫn tồn tại nhưng không phải full-cycle workflow thứ hai.

**Source quan trọng:** workflow/cycle.py điều phối, target, repeat, controls; workflow/pickdrop.py xử lý START, B091, final pose, count và REAL gate; workflow/drop.py chứa StackStore/P010; workflow/state_machine.py chứa journal/instance lock. Robot transport/variables ở robot/; vision ở vision/; JOB local ở jobs/PICKDROP.JBI. API: app.py; HMI: templates/index.html, static/app.js, static/style.css; defaults: config/config.json.

**Bất biến:** giữ calibration/model, ảnh calibration, thuật toán/offset XY, ID/cursor, RX/RY/RZ/Z_SAFE, Decimal Z Drop, metadata P009 và count chống trùng cycle ID. READY chốt không bị re-detect hoặc loại chỉ vì tuổi; vẫn kiểm provenance/connection/config/calibration và guard. PC không ghi B090/B091/P003/P008/P009/P011. Ngoại lệ tốc độ theo yêu cầu mới: ô SPEED ghi I000 có guard/readback, số nguyên 0–5000, đơn vị 0.1 mm/s; không đổi lệnh V=I000 trong JOB. Không retry command unknown hoặc tự resume/HAND OFF sau lỗi.

**REAL hiện đã mở theo xác nhận người vận hành:** bỏ JUMP *TICK; P011 là IMOV theo BASE, X/Y giữ nguyên và Z tăng 200 mm. Config lưu xác nhận/frame/SHA256 JOB; backend đọc vector P011 thực tế, không hardcode +200 để kiểm final pose. Đây không phải chứng nhận an toàn: B091 reset/success, đúng JOB, RUNNING OFF, pose/telemetry/readiness vẫn bắt buộc. Báo cáo hiện có **253 test offline PASS**, browser HOME và SPEED MOCK PASS; chưa có nghiệm thu REAL mới của PICKDROP trong bộ bằng chứng này.

**Tiếp tục phát triển:** app.py mặc định REAL, không có CLI --mock. Mở server không tự CONNECT/START nhưng factory có ghi journal/lock. MOCK dùng create_app(mock=True)/test harness. Dữ liệu tách data/real và data/mock; không xóa/reset REAL journal để test qua. HOME chỉ gọi JOB HOME; sau bàn giao cần stopped + RESET/ACK về IDLE. Đọc source/test liên quan trước patch, cập nhật bằng chứng và tài liệu này; không dùng log cũ, flag cấu hình hay MOCK để khẳng định trạng thái vật lý hiện tại.

## 1. Phạm vi và bản đồ source

Đây là tài liệu handoff chính của source hiện tại; các bản trong docs/ là tham chiếu/lịch sử. Khi khác nhau, ưu tiên source mới. Không giữ flow điều phối cũ như một phương án full cycle hợp lệ.

| Thành phần | File chính | Trách nhiệm |
|---|---|---|
| Entry/API | [app.py](app.py), [core/launcher.py](core/launcher.py) | Factory, routes, lease; mở Edge sau server ready, không tự kết nối robot |
| Config/path | [core/settings.py](core/settings.py), [config/config.json](config/config.json) | Root-owned paths, defaults, settings validation, fingerprint |
| Điều phối | [workflow/cycle.py](workflow/cycle.py) | Cycle(PickDropWorkflow, DropWorkflow), target, batch, HOME/HOLD/manual |
| PICKDROP | [workflow/pickdrop.py](workflow/pickdrop.py) | REAL contract, START một lần, B091, final pose và count |
| Persistence/Z Drop | [workflow/drop.py](workflow/drop.py), [workflow/state_machine.py](workflow/state_machine.py) | StackStore, P010, journal, OS instance lock |
| Guards | [workflow/safety_checks.py](workflow/safety_checks.py) | Ready/workspace/reached, REAL/manual/HOLD/servo |
| TCP/robot | robot/{protocol,controller,position_variable,job_control,cartesian,status,io,vacuum,mock}.py | Framing, variables, telemetry, JOB/I/O, simulator |
| Vision | vision/{camera,detector,calibration,tracker,stability,pipeline}.py | MVS → YOLO → BASE → ID/stability → preview/prefetch |
| JOB/HMI/assets | jobs/*.JBI, templates/, static/, models/, calibration/ | Controller programs, UI, model/calibration |

Một cycle worker sở hữu cả batch; vision worker chỉ chuẩn bị ảnh/READY, SDK có acquisition thread; telemetry thread chỉ đọc. Các component có RLock; _WIRE_LOCK serialize TCP toàn process. Manual HAND chỉ dùng worker riêng khi guard cho phép, không điều khiển song song trong full cycle.

## 2. Workflow SINGLE JOB PICKDROP

~~~mermaid
flowchart TD
    A["START HMI / SYSTEM_CHECK"] --> B["YOLO + tracker + stable target hoặc READY"]
    B --> C["Tính XY; LOADV + SAVEV P002"]
    C --> D["StackStore + SAVEV P009; LOADV + SAVEV P010"]
    D --> E["Guards + RJDIR + kiểm lại biến; START PICKDROP một lần"]
    E --> F["NX100 chạy JOB; Python đọc trạng thái/JOB/B091"]
    F --> G{"B090: đã thấy 0 rồi 1?"}
    G -->|Có| P["Vision prefetch; chốt READY lượt sau"]
    G -->|"Bỏ lỡ reset / không hợp lệ"| Q["Không tin 1 cũ; fallback vision"]
    F --> H{"B091 + JOB + RUNNING + pose + guards"}
    H -->|"2 / invalid / timeout / lỗi"| X["FAULT hoặc HOLD; không count/repeat"]
    H -->|"0 hoặc chưa đủ completion"| F
    H -->|"0 rồi 1; PICKDROP stopped; pose hợp lệ"| I["StackStore.confirm cycle ID; COMPLETE"]
    I --> J{"repeat_until_empty?"}
    J -->|Có| B
    J -->|Không| K["Chờ ACK"]
    B -->|"Rỗng đủ frame/dwell"| L["COMPLETE / no_boards"]
~~~

Chuỗi gọi: _run_batch → _run_once('full_cycle') → _target → _prepare_pick_variable → _prepare_drop_variable → _start_pickdrop → _wait_pickdrop_complete → _confirm_pickdrop.

| Giai đoạn | Hợp đồng |
|---|---|
| START admission | IDLE, không busy/pending/outcome_unknown; request ID mới, revision đúng; robot/camera có dữ liệu mới, robot ready và laser OFF |
| Trước START JOB | Receipt P002/P010 verified cùng cycle/connection; readback lại cả hai, metadata live, prefetch identity, readiness/laser; RJDIR có PICKDROP |
| Dispatch | Persist intent trước control; chỉ một START; ACK chỉ là accepted |
| JOB đang chạy | Không đòi RUNNING OFF tại ranh giới gắp/thả nội bộ; Python không chen chuyển động |
| Kết thúc/lặp | Chỉ sau verified completion/count; kiểm ready/laser OFF, tạo cycle ID mới |
| Hết ván | Frame mới không có detection đủ stable_frames và empty_seconds → COMPLETE/no_boards; không START thêm |

Các stage xy_only/moveunti_only/vacuum_only, helper MOVEUNTI và state cũ chỉ còn commissioning/compatibility. end_after_moveunti_start, z_completion, config.job và drop.after_h2off_move không đổi full cycle về flow cũ.

## 3. Hợp đồng JOB, biến và cảm biến

Nguồn: [PICKDROP.JBI](jobs/PICKDROP.JBI), cấu hình I/O và adapter. Giá trị export là tham chiếu local, không thay đọc biến live NX100.

~~~text
NOP → B090=0, B091=0
MOVL P002 V=I000
IMOV P003 V=23.0 BF UNTIL IN#(1)=ON → laser OFF thì FAULT
HAND 2 ON → I001=0 → kiểm vacuum/TIMER/INC I001 → OK hoặc FAULT
IMOV P008 V=I000 → MOVL P009 V=I000 → B090=1
MOVL P010 V=I000 → HAND 2 OFF → TIMER 1.00 s
IMOV P011 V=I000 → B091=1 → END
FAULT: B091=2 → PAUSE → quay lại HALT
~~~

| Biến/JOB | Vai trò | Quyền PC / lưu ý |
|---|---|---|
| P002 | Pose tiếp cận pick: XY vision, Z_SAFE, RX/RY/RZ settings | LOADV/SAVEV; metadata posture/tool từ RPOSC |
| P003 | Vector tìm laser; export (0,0,-600,0,0,0) | **IMOV BF UNTIL**; PC không sửa |
| P008 | Nâng sau hút; export (0,0,+300,0,0,0) | **IMOV**, không đổi thành MOVL/PC move |
| P009 | Pose tham chiếu thả; export Z=600 | JOB MOVL tới đây; Python chỉ SAVEV, giữ mọi trường trừ Z khi tạo P010 |
| P010 | Pose thả theo stack | LOADV/SAVEV trước START, không tạo từ P002 |
| P011 | Vector rút cuối; export (0,0,+200,0,0,0) | **IMOV**; operator xác nhận BASE; PC đọc vector thật để tính final target |
| I000 | Tốc độ cho các V=I000, đơn vị 0.1 mm/s | SPEED trên HMI đọc live và ghi có guard/readback; không thay lệnh JOB |
| I001 | Bộ đếm kiểm vacuum | JBI reset 0, TIMER 0.10 s rồi INC; FAULT nếu >=10 trước kiểm OK |
| B090 | 0 reset, 1 sau MOVL P009 | JBI sở hữu; PC chỉ SAVEV 0,90 để prefetch |
| B091 | 0 pending, 1 success checkpoint, 2 FAULT | JBI sở hữu; PC chỉ SAVEV 0,91; số khác invalid |
| HOME | JOB riêng: MOVJ P000 VJ=15.00 PL=0 | Chỉ START HOME; không sửa pose/tốc độ JOB |
| H1ON/H1OFF/H2ON/H2OFF | JOB HAND manual | Độc lập; full cycle không tự START |

JUMP *TICK đã được bỏ theo xác nhận vận hành. CHECK hiện rơi xuống VERIFY kể cả khi hai sensor OFF; mỗi vòng chờ 0.10 s và tăng I001. JUMP tới VERIFY khi sensor ON vẫn phải qua bước chờ. Ngưỡng là **10 vòng**, không phải phép đo chính xác 1 s trên máy thật; thời gian thi hành và thứ tự kiểm >=10 cần được tính khi nghiệm thu.

| Tín hiệu | Địa chỉ/backend | Ý nghĩa và giới hạn |
|---|---|---|
| Laser IN#(1) | IOREAD 20050,8, bit0, active high | Guard OFF trước START; JBI tìm Z và xử lý fault |
| Vacuum IN#(2)/IN#(3) | Contacts20051/20052, bit1/2 cùng byte | JBI chấp nhận **sensor1 OR sensor2**; HMI hiển thị từng input |
| Relay/output1/2 | IOREAD 30050,8, contacts30050/30051 | Khác pressure; output ON không chứng minh vật đã bám/nhả |
| HAND2 ON/OFF | Lệnh nội bộ PICKDROP | JOB sở hữu hút/nhả, không START H2ON/H2OFF từ full cycle |
| Network I/O22xxx | Adapter alternative | REAL hiện thiếu start/bits/owned-byte, không dùng thay HAND JOB |

B091=2 là fault JOB chung, có thể do laser/vacuum; không đủ thông tin phân loại chính xác. Không tự reset byte, HAND OFF hoặc START lại.

## 4. P002/P010, final pose và StackStore

| Bước | Hợp đồng source |
|---|---|
| P002 | pick_position serialize 3 decimals; body 1,0,X,Y,Z,RX,RY,RZ,posture,tool; LOADV 4,2,... rồi SAVEV 4,2 |
| Verify P002 | Mỗi tọa độ/góc lệch tối đa Decimal0.001; trường khác phải bằng nhau; kiểm lại metadata RPOSC đầy đủ trước START |
| P010 | SAVEV4,9 → with_z chỉ thay Z body → LOADV4,10,... → SAVEV4,10 so Decimal mọi trường |
| Guards | Pose hữu hạn/workspace; Z_drop <= drop.safe_z_mm + xyz_tolerance; hai receipt verified đúng cycle/connection |
| Final pose | SAVEV4,11; final.xyz=P010.xyz+P011.xyz theo BASE đã xác nhận; orientation giữ P010; chặn P011 rotation khác0 |
| Count | JOB/B091/stopped/pose đúng đủ settle samples, đọc kiểm lại ngay trước StackStore.confirm; ACK/B090 không là completion |

~~~text
Z_drop = first_drop_z_mm + stacked_count * board_thickness_mm
Default: 330 + n*12 mm → ba tấm đầu 330, 342, 354 mm.
~~~

Decimal tránh drift nhị phân; snapshot trả float. P009 giữ XY/orientation/posture/tool, không lấy pick pose hay P010 cũ. P011 được đọc thật, không hardcode Z+200 và không phát PC lift.

StackStore ở <journal.parent>/drop_stack.json lưu geometry/count, completed_drop_cycle_ids, last_completed_drop_cycle_id và last_picked_id. confirm(cycle_id,picked_id) chống cộng trùng qua restart; cursor chỉ đổi sau commit. Ghi temp→flush/fsync→os.replace→commit memory. File hỏng gây lỗi, không tự reset. Manual set_count/reset giữ lịch sử chống trùng/cursor.

Receipt thành công: count_basis='pickdrop_completed', board_release_confirmed=False. Đây là kết quả chương trình được xác minh, **không chứng minh ván đã rời đầu hút**. API confirm cũ còn tương thích, không là bước cần của full cycle thành công.

Chỉnh count: IDLE/COMPLETE, không busy, stopped nếu connected; int thật0..9007199254740991. Thickness >0 và <=1000 mm, tối đa3 decimals; Z0 hữu hạn/tối đa3 decimals, save chung kiểm workspace và <=DROP SAFE. Đổi geometry không reset count; chiều cao vượt safe bị chặn khi chuẩn bị P010.

## 5. B090 prefetch và B091 completion

| Tín hiệu | Chấp nhận | Không được suy ra |
|---|---|---|
| B090 | Arm sau START ACK, poll ngay; thấy 0 rồi1 cùng cycle/generation/connection, RJSEQ=PICKDROP | 1 cũ không trigger; B090 không chứng minh đã thả xong |
| B090 lỡ/lỗi | Prefetch INVALID, clear preparation, fallback stability | Không ghi0 hoặc tạo reset evidence từ ACK |
| B091=0 | Reset phải được quan sát sau START cho cycle mới; tiếp tục chờ | RUNNING OFF cùng0 không phải success |
| B091=1 | Sau reset; START accepted, đúng JOB, stopped, pose và guards đầy đủ | ACK/1/stopped riêng lẻ không đủ count |
| B091=2/khác | FAULT/invalid, không count/repeat; kiểm tra thủ công | Không tự đoán sensor gây lỗi |
| B091 1→0 | FAULT vì reset lại sau checkpoint | Không gộp hai lần chạy thành một cycle |

Parser nhận integer byte0..255, malformed/out-of-range gây lỗi. _sample_pickdrop_result đọc uncached/kiểm identity và JOB. _wait_pickdrop_complete yêu cầu telemetry không read_errors, readiness/workspace, B091 hợp lệ, RUNNING=false, final pose reached đủ settle_samples, không pending/unknown. _confirm_pickdrop đọc lại trước count.

B090 do cycle worker poll khi WAITING_B090, không có thread checkpoint thứ hai. Trigger gọi prepare_next_pick(after_id,cycle_id), chỉ lấy capture stamp sau trigger. Identity gồm toàn config fingerprint, calibration object, bytes T/K/distortion và nội dung file Eye-to-Hand.

- READY giữ target/pose đầu tiên ổn định; frame sau không cập nhật lại tọa độ.
- Sau cycle nguồn count, kiểm last_completed_drop_cycle_id, cursor, generation/identity; lấy READY một lần, không sample/YOLO xác nhận lại hoặc loại chỉ do tuổi READY.
- History chưa READY vẫn phải mới theo policy và cần frame mới xác nhận; mất camera/context/config/calibration thì clear/fallback.
- Giả định latch: sau B090 tay đã tới khu DROP, khu PICK ổn định. Đây không phải hardware interlock.
- Nhiều socket/vòng làm poll chậm hơn poll_seconds. Lỡ reset có thể fallback B090 hoặc FAULT B091; chưa có benchmark latency REAL.

Log: PICKDROP_STARTED, B090_RESET_CONFIRMED, B090_TRIGGERED, PREFETCH_STARTED, NEXT_TARGET_READY, PICKDROP_COMPLETED, NEXT_TARGET_LOADED, NEXT_PICKDROP_SENT, PREFETCH_INVALID.

## 6. Vision, tracking và XY/calibration

**Acquisition:** MVS chọn đúng một camera theo IP; copy/release buffer SDK, chuyển BGR, dùng SDK frame number/capture stamp. Không resize đầu vào để khớp calibration; preview resize riêng không thay hình học inference.

**YOLO/tracking:** models/best.pt; confidence0.5, imgsz960, device="0" theo config (không khẳng định GPU có sẵn). Tâm bbox ảnh gốc đi vào calibration. Tracker ghép cùng class bằng IoU/khoảng cách và tỷ lệ kích thước, ambiguous→uncertain. Track mất tạm giữ ID10 s nhưng không phát detection cũ; arrivals cùng frame cấp ID trái→phải rồi trên→dưới; instance đổi khi ID được cấp lại.

StableTarget chọn ID lớn hơn cursor trước rồi quay vòng nhỏ nhất; bỏ uncertain/nonfinite/low confidence. Config hiện cần >=5 frame khác nhau và >=1 s, XY cách mọi sample <=3 mm, frame age<=2 s, gap<=0.5 s. Đổi ID/instance, lệch XY hoặc gap làm reset history. Cùng XY được phép gắp tiếp; không blacklist XY; HMI polling không tạo frame mới. Policy tuổi READY riêng ở mục5.

**Hệ trục:** camera OpenCV +X phải/+Y xuống/+Z trước camera; T_base_camera định nghĩa Camera→robot BASE, translation mm. Orientation robot lấy settings, không suy từ ảnh.

~~~text
u=(x1+x2)/2; v=(y1+y2)/2
(xn,yn)=cv2.undistortPoints((u,v),K,D)
T_base_camera=[R|t]; o=t; d=R*[xn,yn,1]^T
lambda=(plane_z_mm-o.z)/d.z
p_raw=o+lambda*d
p_out=p_raw+[280,215,0]^T; p_out.z=plane_z_mm
pick_pose=(p_out.x,p_out.y,z_safe,rx,ry,rz)
~~~

Offset BASE **X+280/Y+215 mm đúng một lần**. Không bắt buộc normalize d. Reject pixel ngoài ảnh, nonfinite, |d.z|<1e-9, lambda<=0, sai image size/K/T/units/transform definition. R trực chuẩn/det≈1, hàng cuối T=[0,0,0,1]; K/distortion giữa hai NPZ phải khớp.

| Asset/tham số | Đối chiếu NPZ bằng allow_pickle=False |
|---|---|
| Image size | 5472×3648; không đổi ROI/kích thước để che mismatch |
| K | fx4674.729315, fy4693.272834, cx2549.936888, cy1973.510994 |
| D (k1,k2,p1,p2,k3) | (-0.2532737023,0.2234199981,-0.0031651037,0.0109485923,0.0634522970) |
| Eye-to-Hand quality | needs_review; 22/22 outlier |
| Translation residual RMS/max | 63.9673 / 129.2721 mm |
| Rotation residual RMS/max | 16.8121 / 38.0122 độ |
| Reprojection RMS/max | 184.3329 / 424.4001 px; PnP riêng RMS2.35375 px |

T làm tròn để đọc, không dùng thay asset:

~~~text
T_base_camera ≈ [
 [ 0.024963289,-0.998402026,-0.050697426,1260.248627],
 [-0.999665076,-0.024584400,-0.008083517,  73.614719],
 [ 0.006824233, 0.050882237,-0.998681344,1390.587877],
 [0,0,0,1]]
~~~

Residual/fit không phải accuracy gắp đã nghiệm thu. Constructor kiểm cấu trúc, không buộc quality passed; status warning/coordinates_verified=false. Không có solver recalibration/training pipeline trong project. Integrity: [asset_hashes.json](docs/asset_hashes.json); không unpickle model để audit tài liệu.

Phân biệt plane_z_mm để tính XY; z_safe tiếp cận P002; Z pick thật do laser/JOB; Z_drop từ StackStore; drop.safe_z_mm là giới hạn chuẩn bị thả. Không thay chúng cho nhau hoặc thêm bù XY từ Z laser sau START.

## 7. TCP ASCII, trạng thái và persistence

Mỗi command mở một socket IPv4 mới tới NX Information Server, mặc định port80; không HTTP/YERC UDP/session persistent. Controller RLock và _WIRE_LOCK serialize giao dịch; không sleep khi giữ transport lock.

~~~text
PC → CONNECT Robot_access\r\n
NX → OK: NX Information Server (...).\r\n
PC → HOSTCTRL_REQUEST <COMMAND> <N>\r\n
NX → OK: <COMMAND>\r\n
PC → <data>\r     # sau ACK nếu có data; N gồm CR
NX → <result>\r   # read; control success phải 0000\r\n
~~~

Printable ASCII, payload<=256 byte; reader xử lý fragment/coalesce, response<=65536 byte. RJDIR đọc tới socket close và kiểm terminator cuối; record đầu không phải toàn directory. Control có thể đã thi hành nhưng thiếu reply→OutcomeUnknown, không retry; reject rõ ràng khác outcome chưa biết.

| Command | Payload/hợp đồng đang dùng |
|---|---|
| RSTATS/RJSEQ/RPOSJ | Không data; flags / JOB,line,step / joint read lúc connect |
| RPOSC | 0,1: 14 trường, 6 pose+posture/tool+6 external axes |
| RJDIR | *; exists(client,name) kiểm đầy đủ danh sách |
| START | Tên explicit PICKDROP/HOME/HAND; MOVEUNTI/DROP chỉ compatibility; cấm START rỗng/resume |
| SAVEV | 4,2; 4,9; 4,10; 4,11; 0,90; 0,91; **1,0 đọc Integer I000** |
| LOADV | Cartesian BASE P002/P010; **1,0,n ghi I000 với n nguyên0..5000**; không ghi byte/Integer khác |
| HOLD/SVON | Chính xác1/0; journal/readback theo control |
| IOREAD/IOWRITE | Byte reads; write chỉ network inputs22010..23287 có mapping/ownership hợp lệ |
| MOVL | Helper17 trường compatibility; full cycle không gọi; MOVJ/SETMJ/RESET/CANCEL ngoài public whitelist |

RSTATS data1: bit3 RUNNING, bit5 TEACH, bit6 PLAY, bit7 command REMOTE; data2: bit1 pendant HOLD, bit2 external HOLD, bit3 command HOLD, bit4 alarm, bit5 error, bit6 servo. HOLD tổng=OR ba nguồn. Telemetry poll danh nghĩa1 s; snapshot quá2 s làm cờ/pose/I/O UNKNOWN, không dùng cache để chứng minh completion.

Journal intent trước control, accepted clear pending, revision tăng/history100 mốc. save/update dùng RLock chung theo path; mỗi lần ghi tạo file tạm riêng cùng thư mục, flush/fsync/đóng handle trước atomic os.replace. Không dùng lại hoặc xóa execution.tmp cũ. Riêng replace gặp PermissionError/WinError5/32/33 được thử tối đa6 lần, backoff20/50/100/200/400 ms; không retry START/MOVL/LOADV. Memory chỉ commit candidate sau ghi thành công; accepted thất bại giữ pending cũ. Lỗi persistence chốt JournalWriteError + FAULT/outcome_unknown trong process; mọi intent/update/save sau đó bị khóa, ACK không xóa latch.

API admission persist nonce riêng không tăng revision. OS InstanceLock vẫn chặn process điều khiển thứ hai cùng journal path; RLock journal không thay OS ownership. Startup không ghi lại journal terminal không thay đổi; journal không đọc được/hỏng gây lỗi và giữ nguyên file, không ghi defaults đè dữ liệu. Nếu mất execution.json nhưng còn file tạm phục hồi thì chặn startup, không tự tạo journal IDLE.

Data mặc định ở data/{real|mock}/{execution.json,execution.lock,drop_stack.json,operator_settings.json}; không chuyển settings/stack giữa mode. Nonterminal/pending khi restart→FAULT/outcome_unknown; không replay/resume. Terminal: IDLE, COMPLETE, JOB_DISPATCHED, HOLD, FAULT. Journal/stack/settings là file riêng, không có transaction nguyên tử với nhau/controller. Persistence fault không bảo đảm dừng chuyển động đã gửi; cần kiểm tra robot trước phục hồi.

## 8. Guards, HOLD và state machine

Main states: IDLE→SYSTEM_CHECK→WAIT_TARGET→TARGET_LOCKED→WRITING_PICK_P002→PICK_P002_WRITTEN→WRITING_DROP_P010→DROP_P010_WRITTEN→START_PICKDROP→PICKDROP_RUNNING→PICKDROP_COMPLETE→STACK_COUNT_UPDATED→COMPLETE. Lỗi/cancel→FAULT/HOLD; repeat tạo cycle ID mới. STEPS chỉ là danh sách định danh, không phải transition validator độc lập.

| Tình huống/control | Hành vi source |
|---|---|
| Readiness | Servo/remote/play=true; teach/alarm/error/hold=false; RUNNING=false trước START/control phù hợp |
| Motion guards | Pose hữu hạn/workspace; reached XYZ<=1 mm, góc modulo360<=0.5°, settle3 theo config |
| Full cycle lỗi sau START | Không count/repeat; unknown; yêu cầu HOLD một lần nếu chưa cancel, không khẳng định HOLD thành công khi mất mạng |
| Software HOLD | Cancel trước chờ lock, clear prefetch, persist HOLD/unknown, HOLD1/refresh; event không ngắt socket đã gửi |
| RESET/ACK | Worker dừng, inspected khi unknown, fresh RUNNING=false→IDLE; không reset alarm/count, bật servo/nhả HOLD/resume |
| UNHOLD | IDLE/no pending/unknown; command_hold=true, external/pendant_hold=false; remote/play/stopped/no teach/alarm/error; confirmed=true→HOLD0 một lần+verify |
| SERVO ON/OFF | IDLE/not busy/no pending, connected và guard live; SVON một lần, verify bit hoặc FAULT/unknown |
| Manual HAND | IDLE/COMPLETE/not busy; remote/no teach/running/alarm/error/hold; không bắt buộc servo ON; đúng JOB và stopped2 mẫu |
| Mapping | HAND không được map tới MOVEUNTI/DROP/PICKDROP/HOME; network phải verified/owned, preserve unrelated bits |
| Crash giữa completion/count | Kiểm tra thủ công, không tự count bù/replay START/nhả hút |

REAL full-cycle gate kiểm JOB local tồn tại/SHA256 khớp, không JUMP nhãn thiếu, dòng hợp đồng P008/P011/B090/B091, operator_confirmed=true và p011_frame=BASE. Config hiện đáp ứng theo xác nhận vận hành. Gate không tải/đọc JOB live controller, không thay nghiệm thu đường đi/I/O/calibration. Legacy commissioning flags false không có nghĩa full cycle đang khóa.

## 9. HMI, HOME và API

HMI có Camera/Robot, status servo/remote/mode/alarm/error/running/hold/laser/vacuum; CONNECT, SERVO toggle, START, HOLD/UNHOLD, RESET và HOME. Robot panel có stack editor/pose. Tám input: Z Drop, thickness, Plane Z, Z Safe, **SPEED (0.1 mm/s)**, RX/RY/RZ; HAND1/2 ON/OFF dưới settings. UNKNOWN hiển thị dấu gạch, không suy là OFF.

SPEED dùng input speed_i000, min0/max5000/step1; LƯU THÔNG SỐ gửi speed_i000 riêng trong POST /api/config, không diễn giải lại settings.movl_speed cũ. Cú pháp đã đối chiếu manual NX100 [HW0483202](docs/HW0483202.pdf), mục SAVEV3-12 và LOADV3-26: type1=Integer, variable0, reply scalar. Ghi đúng một LOADV1,0,n rồi SAVEV1,0 để so chính xác. Chỉ khi connected, IDLE/not busy/no pending/unknown, remote, no teach/running/hold/alarm/error; servo OFF vẫn cho phép ghi biến khi các guard khác đạt. Mất reply, đọc lỗi hoặc mismatch→FAULT/unknown/error, không retry/không báo thành công, không lưu các settings khác trong cùng yêu cầu.

Telemetry robot.speed_i000/speed_error đọc từ controller; polling cập nhật HMI kể cả thay đổi trên pendant, không lấy từ file Python. Khi nhập chưa lưu thì giữ giá trị đang sửa; khi mất kết nối/không đọc được thì ô trống và disabled. Lỗi riêng đọc SPEED không thêm vào read_errors của motion, không đổi guard B090/B091. Getter đọc Integer signed16, setter chỉ cho phép0..5000. Simulator có I000 riêng, không mở socket REAL.

State/preview polling500 ms khi không pending; UUID+revision, không retry. Stack dirty khóa START; Esc hủy dirty; nhập0+LƯU chỉnh count, RESET không xóa stack. Settings save khi IDLE; form không bị polling ghi đè liên tục. Notification tự ẩn15 s nếu nội dung không đổi, không clear lỗi backend.

Fullscreen icon32 px di chuyển node thật bằng markers sang hai page100dvh: Camera/Robot và Settings/HAND; Scroll Snap. Exit/Esc khôi phục DOM/input/scroll. Width<=1000, height<=700 hoặc overflow chuyển cuộn tự do. Chỉ thay layout local, không điều khiển robot.

**HOME:** IDLE/COMPLETE, not busy/pending/unknown, connected/ready và RJDIR có HOME; không cần camera. Persist intent/START HOME một lần→JOB_DISPATCHED, completion_verified=false. Không ghi P000, không đổi VJ=15.00, không PC move. Stopped rồi RESET/ACK về IDLE; START/HOME/HAND/servo khóa lúc bàn giao. Mất reply→FAULT/pending/unknown, không retry. ACK START không xác nhận đạt HOME vật lý.

| API | Vai trò |
|---|---|
| GET /, /static/*, /api/state, /api/preview.jpg, /api/laser/status | HMI, snapshot, JPEG, ON/OFF/UNKNOWN |
| POST /api/control/claim, /api/system/connect | Lease; connect robot rồi camera, đợi frame đầu trước START |
| POST /api/cycle/start | Default full_cycle, một worker/batch |
| POST /api/robot/home, /api/robot/servo-on, /api/robot/servo-off | HOME handoff; servo explicit+verify |
| POST /api/cycle/hold, /api/cycle/acknowledge, /api/cycle/release-hold | HOLD; ACK inspected; UNHOLD confirmed |
| POST /api/vacuum/{channel} | Channel1/2, on boolean; manual HAND |
| POST /api/config, /api/drop/count | Settings/drop_settings; chỉnh count |
| POST /api/drop/settings, /api/drop/reset, /api/drop/confirm | Compatibility, reset/confirm cần xác nhận; UI không có nút riêng |

POST trừ claim cần cookie lease một HMI30 s, request_id8..100 chưa dùng/revision đúng; HOLD bỏ stale check. HttpOnly/SameSite strict, TrustedHost/allowed_clients, JSON, Origin cùng host nếu có; LAN cần token>=24 ký tự+allowlists. HTTP409 RuntimeError,422 ValueError,423 lease,403 token/client/origin,415 content type. No-store/nosniff/CSP self. Không có SSO/TLS hoặc Web Viewer riêng.

## 10. Cấu hình và tiếp tục phát triển

| Nhóm | Default/file hiện tại; không phải process/thiết bị live |
|---|---|
| Network/web | NX100192.168.255.1:80, timeout3 s; camera192.168.255.3; HMI127.0.0.1:8011 |
| Pick settings | planeZ0, Z_SAFE500 mm; RX180/RY0/RZ90 độ; settings.movl_speed350 mm/s chỉ còn compatibility PC MOVL |
| Drop | P010, safe600, first330, thickness12 mm, job_timeout120 s |
| Vision | confidence0.5; stable5 frame/1 s/3 mm; age2 s/gap0.5 s; imgsz960/device="0" |
| Motion | timeout120, target_timeout30, legacy job_timeout60 s; poll0.1 s, XYZtol1 mm, angleTol0.5°, settle3 |
| Repeat | repeat_until_empty=true, empty_seconds2 |
| I/O/HAND | Laser20050 bit0; pressure20051/52; outputs30050/51; HAND JOB, timeout10 s |
| REAL contract | operator_confirmed=true, p011_frame=BASE; SHA256 JOB khớp local export |
| Overrides | Operator settings/StackStore cùng thư mục journal ghi đè defaults theo mode |

SPEED trên HMI **điều khiển trực tiếp I000** sau ghi–verify. Legacy settings.movl_speed trong Python vẫn giữ đơn vị mm/s cho helper MOVL cũ; không dùng làm giá trị hiển thị/khởi tạo I000. HOME.JBI dùng VJ riêng, không đổi theo SPEED. Workspace không phải collision planner/interlock; z_safe không chứng nhận cả đường đi tránh va chạm.

owned() resolve resources trong project root, không dùng đường dẫn dự án cũ. Dependencies chia requirements/{base,real,dev}.txt và lock inventory; REAL cần MVS SDK ngoài+Ultralytics/PyTorch; MOCK không nạp YOLO/SDK. Lock file không chứng minh version đã cài. Không có validator tổng thể đảm bảo mọi config nhất quán.

CLI app.py mặc định REAL; --no-browser, --port và --real compatibility; không --mock. Startup mở Edge sau server ready, không tự CONNECT/START. Factory đọc calibration/tạo-lưu journal/lock/stack; không chạy factory chỉ để audit tài liệu. Process đã load có thể khác config trên disk; lần docs này không restart backend.

Khi sửa tiếp: đọc source/test của phần sửa, kiểm JBI/contract trước đổi motion; giữ công thức/metadata/identity/READY/guard/data. Với thay đổi thực thi, dùng journal tạm và MOCK/FakeSocket. Các lệnh sau chỉ hướng dẫn, **không chạy trong lần sửa tài liệu này**:

~~~powershell
.venv\Scripts\python.exe -B -m tools.run_tests
.venv\Scripts\python.exe -B -m tools.browser_home_smoke
~~~

Unit/integration runner chặn socket.create_connection mặc định; wire tests thay bằng FakeSocket riêng. Browser HOME dùng private MOCK server/chặn robot TCP trong subprocess. tools/browser_smoke.py cũ còn giả định lịch sử, không dùng nó để khẳng định regression toàn UI hiện tại.

## LAST UPDATE

**Ngày:** 28/10/2026, Asia/Bangkok. **Cập nhật mới nhất:** thay MOVL speed bằng SPEED Integer I000; mở hẹp LOADV/SAVEV1,0, guard/write/readback/no-retry, telemetry và HMI live; bổ sung tests/test_speed_i000.py. Giữ nguyên JOB V=I000, bố cục/nút, XY/Z/count và dữ liệu vận hành; không chạy thử REAL. Bản vá journal trước đó vẫn giữ nguyên.

Chẩn đoán WinError5: log lỗi tại replace cả khi lưu intent trước LOADV và khi poll B091; cơ chế cũ dùng execution.tmp cố định, save không tự giữ lock và không retry. ACL đọc được cho phép Modify, file không ReadOnly, probe DELETE access hiện thành công. Chưa xác định holder tại đúng thời điểm lỗi; không quy kết antivirus/nhiều process khi chưa có bằng chứng. Test tái hiện bằng handle Windows không cấp FILE_SHARE_DELETE; retry hữu hạn xử lý khóa tạm, khóa kéo dài fail-closed. Không sửa ACL, kết thúc process, xóa execution.tmp/lock hoặc reset journal/stack REAL.

| Bằng chứng | Kết quả hiện có |
|---|---|
| Đã code | Một START PICKDROP; P002/P010 trước START; B090 prefetch; B091/final pose completion; count/repeat; persistence/guards; HOME UI/API |
| Operator xác nhận | Bỏ JUMP TICK; P011 IMOV BASE, X/Y giữ nguyên/Z+200; REAL đã mở trong config/gate |
| Test offline 28/10 | [test_result.json](logs/reports/tests/test_result.json): **253/253 PASS**, 0 failure/error/skipped, 47.83 s; [chi tiết](logs/reports/tests/test_output.txt). Bao gồm 16 journal tests và 16 I000 tests về wire/type/range, guards, readback, no-retry, restart và API |
| Browser SPEED MOCK | [speed_i000_browser_result.json](logs/reports/speed_i000_browser_result.json): PASS; hiển thị I000 live/thay đổi pendant, giữ input đang sửa, LƯU ghi–verify, lost reply không retry/không báo thành công; giữ 8 input/5 nút/6 ô status, không JS error |
| Browser HOME MOCK | [result.json](logs/reports/home_browser/result.json): PASS, HOME→JOB_DISPATCHED, ACK→IDLE, tablet không tràn ngang/không JS error |
| Chưa nghiệm thu REAL mới | Không có bằng chứng commissioning PICKDROP máy thật trong bộ báo cáo này; không chứng nhận an toàn/accuracy từ MOCK/config |

Nền tảng trước bản vá journal: flow PICKDROP, ghi biến/count đúng thời điểm, B091/P011 adapter/HOME, age2/device0 và phân biệt HMI MOVL speed/I000. Bản vá journal 28/10 không thay các thuật toán, workflow hoặc chức năng điều khiển này.

**Vấn đề còn tồn tại:**

- Calibration needs_review/residual lớn; cấu trúc hợp lệ không chặn quality kém. Cần nghiệm thu accuracy, gá camera/tool, đường đi/collision và I/O trên máy.
- Hash JOB local không chứng minh controller có đúng bản đó; P011 BASE dựa xác nhận vận hành. Adapter chỉ hỗ trợ translation, không incremental rotation.
- Nhiều socket có thể làm lỡ B090/B091=0; fallback/fault là hành vi bảo thủ, chưa có benchmark giảm cycle time REAL.
- B091/HAND completion không chứng minh giữ/thả vật lý; backend chưa bảo đảm giám sát mất vacuum liên tục khi vận chuyển.
- Journal/stack/operator-settings/controller không transaction chung; crash/disk error/unknown cần kiểm tra thủ công, không tự reset REAL data.
- Software HOLD/OS lock không thay E-stop/hardware interlock; mất mạng không chứng minh đã dừng. Báo cáo test chưa gắn manifest hash mọi source; đọc lại phần liên quan trước patch.

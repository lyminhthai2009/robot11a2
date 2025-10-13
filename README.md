# robot11a2
nhom NASA
### **Cách Tinh Chỉnh Thông Số Kp, Ki, Kd Để Robot Chạy Mượt Hơn**

 (`Kp=35, Ki=0.05, Kd=25`) là điểm khởi đầu tốt, nhưng để robot của bạn đạt hiệu suất cao nhất, bạn cần tự mình tinh chỉnh chúng.

**Chuẩn bị:**

1.  Kết nối robot với máy tính.
2.  Mở **Serial Monitor** trong Arduino IDE (đặt baudrate là 115200). Bạn sẽ thấy các giá trị `Error`, `P`, `I`, `D`... được in ra liên tục.
3.  Chuẩn bị một sa hình có cả đoạn thẳng và đoạn cong.

**Thực hiện theo từng bước sau:**

#### **Bước 1: Tinh chỉnh Kp**

*   **Mục đích:** Giúp robot phản ứng đủ nhanh với vạch đen.
*   **Cách làm:**
    1.  Đặt `Ki = 0` và `Kd = 0`.
    2.  Bắt đầu với `Kp` nhỏ (ví dụ: `Kp = 10`). Đặt robot lên line. nó đi rất chậm và lười biếng, rẽ rất yếu.
    3.  Tăng dần `Kp` lên (ví dụ: 15, 20, 25,...). robot bám line tốt hơn, phản ứng nhanh hơn ở các khúc cua.
    4.  Tiếp tục tăng `Kp` cho đến khi robot bắt đầu **rung lắc/dao động (zic-zac) mạnh** khi đi trên đường thẳng. Đây là dấu hiệu `Kp` quá cao.
    5.  Lúc này, **giảm `Kp` xuống khoảng 60-70%** giá trị gây rung lắc.

*   **Kết quả mong muốn:** Robot bám theo line một cách dứt khoát nhưng vẫn còn hơi rung nhẹ.

#### **Bước 2: Tinh chỉnh Kd (Thành phần Vi phân)**

*   **Mục đích:** Giảm rung lắc, làm robot "mượt" hơn, giống như một bộ giảm xóc.
*   **Cách làm:**
    1.  Giữ nguyên giá trị `Kp` tốt nhất vừa tìm được. `Ki` vẫn bằng 0.
    2.  Bắt đầu tăng dần `Kd` từ 0 (ví dụ: 5, 10, 15,...).
    3.  Robot **hết rung lắc** và chạy trên đường thẳng một cách mượt mà. Ở các khúc cua, nó sẽ ít bị "văng" ra ngoài hơn. `Kd` có tác dụng "phanh hãm" khi robot lao về tâm quá nhanh.
    4.  Nếu `Kd` quá cao, robot sẽ di chuyển rất "lì", phản ứng chậm và có thể bị trượt khỏi các khúc cua gấp.

*   **Kết quả mong muốn:** Robot chạy êm, ổn định trên đường thẳng và vào cua mượt mà, không bị dao động quá mức.

#### **Bước 3: Tinh chỉnh Ki (Thành phần Tích phân)**

*   **Mục đích:** Khử các sai số nhỏ tồn tại trong thời gian dài. Giúp robot chạy **chính xác ở tâm** của vạch đen trên những đoạn đường thẳng dài.
*   **Cách làm:**
    1.  Giữ nguyên `Kp` và `Kd` đã tinh chỉnh.
    2.  Nếu thấy trên đoạn thẳng, robot vẫn có xu hướng chạy hơi lệch về một bên, hãy bắt đầu tăng `Ki` với một giá trị **rất nhỏ** (ví dụ: `0.01`, `0.02`, `0.05`...).
    3.  Thành phần `I` sẽ từ từ tích lũy sai số và tạo ra một lực điều chỉnh nhỏ để đẩy robot về đúng tâm.
    4.  **Cảnh báo:** `Ki` là thành phần rất nhạy cảm. Nếu `Ki` quá lớn, nó sẽ gây ra dao động lớn và làm hệ thống mất ổn định. Thường thì bạn chỉ cần một giá trị `Ki` rất nhỏ.

*   **Kết quả mong muốn:** Robot chạy thẳng tắp và chính giữa vạch đen trên các đoạn thẳng.
